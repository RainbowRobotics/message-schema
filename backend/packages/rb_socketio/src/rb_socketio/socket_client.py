import inspect
import itertools
import re

import socketio

from .socket_client_router import (
    RbSocketIORouter,
)


class RBSocketIONsClient(socketio.AsyncClient):
    def __init__(self, prefix_event_name="", *a, **args):
        self.prefix_event_name = prefix_event_name.strip("/")
        self.on_event_map: set[str] = set()
        super().__init__(*a, **args, request_timeout=30)

    def _on_connect(self):
        print("on_connect", flush=True)

    def _check_event_name(self, event):
        return (
            event
            if event.startswith(f"{self.prefix_event_name}/")
            else f"{self.prefix_event_name}/{event}"
        )

    def _expand_event_patterns(
        self, pattern: str, path_params: dict[str, list[str]] | None
    ) -> list[str]:
        tokens = re.findall(r"{(\w+)}", pattern)
        if not tokens:
            return [pattern]

        if not path_params:
            raise ValueError(f"'{pattern}'에서 path_params가 없습니다. path_params를 제공해주세요.")

        missing = [t for t in tokens if t not in path_params or not path_params[t]]
        if missing:
            raise ValueError(f"'{pattern}'에서 {missing} 형태의 토큰에 대한 값이 없습니다.")

        vals = [
            (
                list(path_params[t])
                if isinstance(path_params[t], list | tuple | set)
                else [str(path_params[t])]
            )
            for t in tokens
        ]
        expanded = []
        for combo in itertools.product(*vals):
            ev = pattern
            for t, v in zip(tokens, combo, strict=False):
                ev = ev.replace(f"{{{t}}}", str(v))
            expanded.append(ev)
        return expanded

    def _extract_params_from_event(self, template: str, expanded: str) -> dict[str, str]:
        pattern = re.sub(r"{(\w+)}", r"(?P<\1>[^/]+)", template)
        m = re.fullmatch(pattern, expanded)

        return m.groupdict() if m else {}

    async def emit(self, event, data=None, **args):
        if not self.connected:
            print("🚫 emit not connected", flush=True)
            return

        event = self._check_event_name(event)
        return await super().emit(event, data, **args)

    async def call(self, event, data=None, timeout=None, namespace=None, **args):
        return await super().call(event, data, namespace=namespace, timeout=timeout, **args)

    async def _trigger_event(self, event, namespace, *args):
        BUILTINS = {
            "join",
            "leave",
            "connect",
            "reconnect",
            "reconnect_error",
            "reconnect_failed",
            "disconnect",
            "message",
        }
        if event in BUILTINS:
            return await super()._trigger_event(event, namespace, *args)

        for on_event_name in self.on_event_map:
            template = self._check_event_name(on_event_name)
            regex = re.sub(r"{(\w+)}", r"(?P<\1>[^/]+)", template)
            m = re.fullmatch(regex, event)
            if not m:
                continue

            path_params = m.groupdict()

            base = list(args)
            if not base:
                base = [{}]
            elif base[0] is None:
                base[0] = {}

            tail = dict(base.pop()) if len(base) >= 2 else {}

            existing = tail.get("__path_params__", {})
            tail["__path_params__"] = {**existing, **path_params}
            new_args = (*base, tail)

            return await super()._trigger_event(template, namespace, *new_args)

        return await super()._trigger_event(event, namespace, *args)

    def socket_include_router(self, router: RbSocketIORouter):
        for r in router.routes:
            self.on_event_map.add(r.event)
            self.on(r.event, **r.kwargs)(r.handler)

    def on(self, event, handler=None, namespace=None, **kwargs):
        parent_on = super().on

        BUILTINS = {
            "join",
            "leave",
            "connect",
            "reconnect",
            "reconnect_error",
            "reconnect_failed",
            "disconnect",
            "message",
        }
        tpl_event = event if event in BUILTINS else self._check_event_name(event)

        def decorator(user_handler):
            sig = inspect.signature(user_handler)
            param_names = set(sig.parameters)

            async def wrapped(*payload):
                if len(payload) >= 2:
                    extra = payload[-1]
                    core = payload[:-1]
                else:
                    extra = {}
                    core = payload

                path_params = extra.get("__path_params__", {}) or {}
                cleaned = {k: v for k, v in path_params.items() if k in param_names}

                if inspect.iscoroutinefunction(user_handler):
                    return await user_handler(*core, **cleaned)
                else:
                    return user_handler(*core, **cleaned)

            # 실제 등록은 부모에게 맡긴다 (namespace만 전달)
            parent_on(tpl_event, handler=wrapped, namespace=namespace)

            self.on_event_map.add(tpl_event)
            return user_handler

        if handler is None:
            return decorator

        return decorator(handler)
