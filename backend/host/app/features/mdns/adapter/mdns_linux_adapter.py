import asyncio
import socket
import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List

from zeroconf import Zeroconf, ServiceInfo, IPVersion
from zeroconf.asyncio import AsyncZeroconf


def _get_primary_ipv4() -> Optional[str]:
    """
    '현재 기본 라우팅으로 나가는' IPv4를 하나 잡는 가장 흔한 방법.
    (실제 통신은 안 하고 커널 라우팅 테이블을 이용)
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None


def _get_primary_ipv6() -> Optional[str]:
    """
    IPv6 기본 주소가 필요하면 비슷한 방식.
    환경에 따라 실패할 수 있어서 optional로 둠.
    """
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        s.connect(("2001:4860:4860::8888", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None


@dataclass
class MdnsConfig:
    """
    [mDNS 설정]
    """

    service_type: str = "_rainbow-robot._tcp.local."
    ttl: int = 4500
    port: int = 8180

    # TXT 항목
    model: str = "unknown"
    robot_serial: str = "unknown"
    api_mode: str = "msa"

    # 광고 주기(초): TTL의 80%처럼
    refresh_ratio: float = 0.8

    # IP 변경 체크 주기(초): 600초
    ip_check_interval: int = 600


class MdnsAnnouncer:
    """
    [mDNS 광고기]
    """
    def __init__(self, cfg: MdnsConfig):
        self.cfg = cfg
        self.zc: Optional[AsyncZeroconf] = None
        self.info: Optional[ServiceInfo] = None

        # 인스턴스 관련
        self.instance_name: str = f"rainbowbot-{cfg.robot_serial}"
        self.instance_fqdn: str = f"{self.instance_name}.{cfg.service_type}"
        self.target_host: str = f"{self.instance_name}.local."

        # 상태
        self.announced = False
        self.last_ip: Optional[str] = None

        # 태스크
        self._refresh_task: Optional[asyncio.Task] = None
        self._ip_task: Optional[asyncio.Task] = None

    def _build_txt(self) -> Dict[bytes, bytes]:
        return {
            b"model": self.cfg.model.encode(),
            b"robot_serial": self.cfg.robot_serial.encode(),
            b"api_mode": self.cfg.api_mode.encode(),
        }

    def _get_current_ip_and_version(self) -> Tuple[Optional[str], IPVersion]:
        """
        ipv4, ipv6 중 하나를 반환
        """
        ip4 = _get_primary_ipv4()
        if ip4:
            return ip4, IPVersion.V4Only

        ip6 = _get_primary_ipv6()
        if ip6:
            return ip6, IPVersion.V6Only

        return None, IPVersion.V4Only

    def _make_service_info(self, ip: str) -> ServiceInfo:
        # zeroconf는 addresses에 "bytes" 리스트를 받음 (IPv4/IPv6)
        # (IPv6는 inet_pton(AF_INET6, ip))
        if ":" in ip:
            addr = socket.inet_pton(socket.AF_INET6, ip)
        else:
            addr = socket.inet_aton(ip)

        print(f"addr: {addr}")
        try:
            info = ServiceInfo(
                type_=self.cfg.service_type,
                name=self.instance_fqdn,
                addresses=[addr],
                port=self.cfg.port,
                properties=self._build_txt(),
                server=self.target_host
            )
            return info
        except Exception as e:
            print(f"error: {e}")
            return None

    async def start(self) -> None:
        # IP 확보
        try:
            ip, ip_ver = self._get_current_ip_and_version()
            if not ip:
                raise RuntimeError("mDNS 시작 실패: 사용 가능한 IP를 찾지 못했습니다.")

            # Zeroconf 생성 (IPv4만 쓰려면 V4Only 고정 가능)
            self.zc = AsyncZeroconf(ip_version=ip_ver, interfaces=[ip])

            # ServiceInfo 생성/등록
            self.info = self._make_service_info(ip)
            print(f"self.info: {self.info}",flush=True)
            print("-1")
            try:
                await self.zc.async_register_service(self.info, ttl=self.cfg.ttl, cooperating_responders=True)
            except Exception as e:
                print(f"error: {e}")
                raise e
            self.announced = True
            self.last_ip = ip

            # zeroconf는 register_service 시점에 기본 광고+응답을 처리함.
            # 그래도 너 코드처럼 "부팅 직후/5초 후/주기적"로 갱신하고 싶으면 update_service를 주기적으로 호출하면 됨.
            self._refresh_task = asyncio.create_task(self._refresh_loop())
            self._ip_task = asyncio.create_task(self._ip_check_loop())
        except Exception as e:
            print(f"error: {e}")

    async def stop(self) -> None:
        if self._refresh_task:
            self._refresh_task.cancel()
        if self._ip_task:
            self._ip_task.cancel()

        if self.zc and self.info:
            # goodbye(bye): unregister가 보통 ttl=0 goodbye를 포함해 처리
            try:
                await self.zc.async_unregister_service(self.info)
            except Exception:
                pass

            await self.zc.async_close()

        self.announced = False

    async def _refresh_loop(self) -> None:
        """
        TTL의 80% 주기로 update_service 호출.
        """
        assert self.zc is not None
        assert self.info is not None

        interval = max(1, int(self.cfg.ttl * self.cfg.refresh_ratio))
        # 부팅 직후 "자발 광고" 느낌을 내고 싶으면 등록 직후 바로 update 한 번
        await asyncio.sleep(0.1)
        try:
            self.zc.update_service(self.info)
        except Exception:
            pass

        # 5초 후 안정화 광고
        await asyncio.sleep(5)
        try:
            self.zc.update_service(self.info)
        except Exception:
            pass

        while True:
            await asyncio.sleep(interval)
            try:
                self.zc.update_service(self.info)
            except Exception:
                # 광고 갱신 실패는 치명적이지 않음
                pass

    async def _ip_check_loop(self) -> None:
        """
        IP 변경 감지: 바뀌면 기존 등록을 bye 처리 후 새 IP로 재등록.
        """
        while True:
            await asyncio.sleep(self.cfg.ip_check_interval)
            if not self.announced:
                continue

            ip, ip_ver = self._get_current_ip_and_version()
            if not ip:
                continue

            if self.last_ip and ip == self.last_ip:
                continue

            # IP 변경 처리
            await self._handle_ip_change(new_ip=ip, ip_ver=ip_ver)

    async def _handle_ip_change(self, new_ip: str, ip_ver: IPVersion) -> None:
        """
        너 코드의:
          - sendByeForIp(old_ip)
          - respondAll() (새 IP로 재광고)
        를 zeroconf 방식으로 구현.
        """
        if not self.zc or not self.info:
            return

        old_info = self.info
        try:
            # 1) 기존 서비스 unregister -> goodbye(bye) 효과
            self.zc.unregister_service(old_info)
        except Exception:
            pass

        # 2) Zeroconf 인스턴스가 ip_version과 매칭이 필요할 수 있음.
        #    IPv4<->IPv6로 바뀌는 환경이면 Zeroconf 자체를 재생성하는 편이 안전.
        try:
            self.zc.close()
        except Exception:
            pass

        self.zc = Zeroconf(ip_version=ip_ver)

        # 3) 새 IP로 ServiceInfo 만들어서 재등록
        self.info = self._make_service_info(new_ip)
        self.zc.register_service(self.info, cooperating_responders=True)

        self.last_ip = new_ip
