from rb_modules.log import rb_log
from .adapter.mdns_linux_adapter import MdnsConfig
from .adapter.mdns_linux_adapter import MdnsAnnouncer
import asyncio

class MdnsService:
    """
    [Mdns 서비스]
    """
    async def start(self):
        """
        [Mdns 서비스 시작]
        """

        # 1) robotSerial 조회
        robot_serial = await self.get_robot_serial()
        robot_model = await self.get_robot_model()

        # 2) MdnsConfig 생성
        config = MdnsConfig(
            service_type="_rainbow-robot._tcp.local.",
            ttl=4500,
            port=8180,
            model=robot_model,
            robot_serial=robot_serial,
            api_mode="rrs",
            ip_check_interval=600,
        )

        rb_log.info("[mdns_service] Mdns 서비스 시작")
        announcer = MdnsAnnouncer(config)
        await announcer.start()
        try:
            while True:
                rb_log.info("[mdns_service] Mdns 서비스 실행 중")
                await asyncio.sleep(3600)
        except KeyboardInterrupt:
            pass
        finally:
            rb_log.info("[mdns_service] Mdns 서비스 종료")
            await announcer.stop()

    async def get_robot_serial(self):
        """
        [Robot Serial 조회]
        """
        return "TESTYUJIN123"
    async def get_robot_model(self):
        """
        [Robot Model 조회]
        """
        return "RBY-test"
