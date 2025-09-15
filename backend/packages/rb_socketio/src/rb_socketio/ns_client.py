import asyncio
import inspect
import itertools
import re

import socketio

from .socket_client_router import RbSocketIORouter


class RBSocketIONsClient(socketio.AsyncClient):
    def __init__(self, prefix_event_name="", *a, **args):
        self.prefix_event_name = prefix_event_name.strip("/")
        super().__init__(*a, **args)

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
                else [path_params[t]]
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
        event = self._check_event_name(event)
        return await super().emit(event, data, **args)

    async def call(self, event, data=None, timeout=None, namespace=None, **args):
        return await super().call(event, data, namespace=namespace, timeout=timeout, **args)

    def socket_include_router(self, router: RbSocketIORouter):
        for r in router.routes:
            self.on(r.event, path_params=r.path_params, **r.kwargs)(r.handler)

    def on(self, event, path_params: dict[str, list[str]] | None = None, **args):
        parent_on = super().on

        if event not in ["join", "leave", "connect", "disconnect", "message"]:
            event = self._check_event_name(event)

        template = event

        def decorator(user_handler):
            expanded_events = self._expand_event_patterns(event, path_params)
            print("expanded_events", expanded_events, args, flush=True)

            sig = inspect.signature(user_handler)
            param_names = set(sig.parameters.keys())
            wants_meta = "meta" in param_names

            for ev in expanded_events:
                token_map = self._extract_params_from_event(template, ev)

                async def wrapped(*payload, __ev=ev, __token_map=token_map):
                    meta = {
                        "event": __ev,
                        "template": template,
                        "params": __token_map,
                        "namespace": args.get("namespace"),
                        "raw_args": payload,
                    }

                    kw = {k: v for k, v in __token_map.items() if k in param_names}
                    if wants_meta:
                        kw["meta"] = meta

                    try:

                        if inspect.iscoroutinefunction(user_handler):
                            return await user_handler(*payload, **kw)
                        else:
                            return user_handler(*payload, **kw)
                    except asyncio.CancelledError:
                        raise
                    except Exception as e:
                        print("socket error:", __ev, e, flush=True)
                        return {"error": True, "code": e.__class__.__name__, "message": str(e)}

                parent_on(ev, handler=wrapped, **args)

            return user_handler

        return decorator
