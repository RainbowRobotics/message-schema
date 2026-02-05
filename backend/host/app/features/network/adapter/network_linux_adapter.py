"""
[NMCLI 어댑터]
"""
import subprocess
import re
import ipaddress
from typing import Optional, Dict, List, Tuple
from rb_flat_buffers.IPC.Network import NetworkT
from rb_modules.log import rb_log
from rb_utils.service_exception import ServiceException
from app.features.network.port.network_port import NetworkPort
from app.features.network.domain.network import Network, NetworkModel
from rb_flat_buffers.IPC.Wifi import WifiT


_ipv4_re = re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b")

def _run_cmd(cmd: List[str]) -> Tuple[int, str, str]:
    p = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return p.returncode, p.stdout, p.stderr


def _parse_ip_and_netmask(ip_with_prefix: str) -> Tuple[Optional[str], Optional[str]]:
    """
    nmcli IP4.ADDRESS는 보통 '192.168.0.10/24' 형태로 파싱 필요
    """
    if not ip_with_prefix:
        return None, None
    try:
        ip_with_prefix = ip_with_prefix.split("|")[0].strip()
        iface = ipaddress.ip_interface(ip_with_prefix)
        return str(iface.ip), str(iface.netmask)
    except ValueError:
        # 혹시 '192.168.0.10'만 오는 등 예외 케이스
        return ip_with_prefix, None

def _get_ipv4_method(ssid: str) -> Optional[str]:
    code, out, err = _run_cmd(["nmcli", "-t", "-g", "ipv4.method", "con", "show", ssid])
    if code != 0:
        rb_log.error(f"[network] _get_ipv4_method ERROR: {err}")
        return None
    if out.strip() == "auto":
        return True
    else:
        return False

def _parse_dns(dns_raw: str) -> List[str]:
    """
    nmcli -g IP4.DNS 는 환경에 따라:
    - 한 줄에 '1.1.1.1,8.8.8.8'
    - 여러 줄로 '1.1.1.1\n8.8.8.8'
    등 다양하게 옴.
    """
    if not dns_raw:
        return []

    candidates = _ipv4_re.findall(dns_raw)

    # 중복 제거(순서 유지)
    seen = set()
    out = []
    for x in candidates:
        if x not in seen:
            seen.add(x)
            out.append(x)
    return out


def _get_devices_status() -> List[Tuple[str, str, str]]:
    """
    nmcli dev status 명령어를 통해 디바이스 상태 조회
    """

    code, out, err = _run_cmd(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "dev", "status"])
    if code != 0:
        rb_log.error(f"[get_devices_status] ERROR: {err}")
        return []
    if not out:
        return []

    # DEVICE, TYPE, STATE 파싱 후 튜플로 반환
    result = []
    for line in out.splitlines():
        dev, typ, state = (line.split(":") + ["", "", ""])[:3]
        if dev:
            result.append((dev, typ, state))
    return result

def _get_ssid(device: str) -> Optional[str]:
    # 1) GENERAL.CONNECTION 조회 -> 활성 프로파일 이름 조회
    code, conn, err = _run_cmd(["nmcli", "-g", "GENERAL.CONNECTION", "dev", "show", device])
    if code != 0:
        rb_log.error(f"[get_ssid] nmcli GENERAL.CONNECTION ERROR: {err}")
        return None
    if not conn or conn.strip() in {"--", ""}:
        return None

    # 2) 활성 프로파일 이름에서 802-11-wireless.ssid 조회 -> SSID 반환 (반환 예 : "mobile_team")
    code, ssid, err = _run_cmd(["nmcli", "-g", "802-11-wireless.ssid", "con", "show", conn])
    if code != 0:
        # rb_log.error(f"[get_ssid] nmcli 802-11-wireless.ssid ERROR: {err}")
        return conn.strip()
    if ssid and ssid.strip() and ssid.strip() != "--":
        return ssid.strip()

    return None


def _build_network(device: str, typ: str) -> Optional[Network]:
    """
    nmcli dev show 명령어를 통해 각 device의 상세정보를 조회 후 반환
    """

    # 1) IPv4 주소/게이트웨이/DNS 가져오기
    code, vals, err = _run_cmd(["nmcli", "-g", "IP4.ADDRESS,IP4.GATEWAY,IP4.DNS", "dev", "show", device])
    if code != 0:
        rb_log.error(f"[build_network] ERROR: {err}")
        return None
    if vals is None:
        return None

    # 2) lines에서 IP4.ADDRESS, IP4.GATEWAY, IP4.DNS 구분
    lines = vals.splitlines()
    ip4_addr = lines[0].strip() if len(lines) > 0 else ""
    ip4_gw   = lines[1].strip() if len(lines) > 1 else ""
    ip4_dns  = "\n".join(lines[2:]).strip() if len(lines) > 2 else ""

    print("ip4_addr: ", ip4_addr, flush=True)
    # 3) IP4.ADDRESS 파싱 후 ip, netmask 반환
    ip, netmask = _parse_ip_and_netmask(ip4_addr)

    # 4) IP4.GATEWAY 파싱 후 gateway 반환
    gateway = ip4_gw or None

    # 5) IP4.DNS 파싱 후 dns 반환
    dns = _parse_dns(ip4_dns)

    # 6) 와이파이 관련 정보 조회 후 ssid 반환
    ssid = _get_ssid(device)
    dhcp = _get_ipv4_method(ssid)

    # 7) 신호강도 조회 후 signal 반환
    signal = wifi_signal_from_proc(device) if typ in {"wifi", "802-11-wireless"} else None

    # 8) Network 객체 반환
    return Network(
        device=device,
        dhcp=dhcp,
        ssid=ssid,
        address=ip,
        netmask=netmask,
        gateway=gateway,
        dns=dns,
        signal=signal
    )

def _build_networkT(device: str, typ: str) -> Optional[Network]:
    """
    nmcli dev show 명령어를 통해 각 device의 상세정보를 조회 후 반환
    """

    # 1) IPv4 주소/게이트웨이/DNS 가져오기
    code, vals, err = _run_cmd(["nmcli", "-g", "IP4.ADDRESS,IP4.GATEWAY,IP4.DNS", "dev", "show", device])
    if code != 0:
        rb_log.error(f"[build_network] ERROR: {err}")
        return None
    if vals is None:
        return None

    # 2) lines에서 IP4.ADDRESS, IP4.GATEWAY, IP4.DNS 구분
    lines = vals.splitlines()
    ip4_addr = lines[0].strip() if len(lines) > 0 else ""
    ip4_gw   = lines[1].strip() if len(lines) > 1 else ""
    ip4_dns  = "\n".join(lines[2:]).strip() if len(lines) > 2 else ""


    # 3) IP4.ADDRESS 파싱 후 ip, netmask 반환
    ip, netmask = _parse_ip_and_netmask(ip4_addr)

    # 4) IP4.GATEWAY 파싱 후 gateway 반환
    gateway = ip4_gw or None

    # 5) IP4.DNS 파싱 후 dns 반환
    dns = _parse_dns(ip4_dns)

    # 6) 와이파이 관련 정보 조회 후 ssid 반환
    ssid = _get_ssid(device)
    dhcp = _get_ipv4_method(ssid)

    # 7) 신호강도 조회 후 signal 반환
    signal = wifi_signal_from_proc(device) if typ in {"wifi", "802-11-wireless"} else None


    # 7) Network 객체 반환
    return NetworkT(
        device=device,
        dhcp=dhcp,
        ssid=ssid,
        address=ip,
        netmask=netmask,
        gateway=gateway,
        dns=dns,
        signal=signal
    )

def wifi_signal_from_proc(device: str) -> Optional[int]:
    """
    /proc/net/wireless 의 link quality(0~70 근처)를 0~100으로 대충 매핑.
    매우 빠름. (스캔 없음)
    """
    try:
        with open("/proc/net/wireless", "r", encoding="utf-8") as f:
            lines = f.read().splitlines()
    except OSError as e:
        rb_log.error(f"[wifi_signal_from_proc] OSError: {e}")
        return None

    for line in lines[2:]:
        # 예: "wlp3s0: 0000   55.  -55.  -256        0      0      0      0      0        0"
        if not line.strip().startswith(device + ":"):
            continue
        parts = line.replace(":", " ").split()
        # parts[2] = link quality (예: "55.")
        try:
            quality = float(parts[2])
            # 보통 0~70 정도로 보이니 0~100으로 스케일
            signal = int(max(0, min(100, round((quality / 70.0) * 100))))
            return signal
        except (IndexError, ValueError) as e:
            rb_log.error(f"[wifi_signal_from_proc] IndexError or ValueError: {e}")
            return None
    return None

class NetworkLinuxAdapter(NetworkPort):
    """
    [Network NMCLI 어댑터]
    """

    async def get_network(self, ssid: Optional[str] = None) -> dict:
        """
        [네트워크 조회]
        - nmcli로 전체 디바이스 상태 조회 후 상세 조회
        - ethernet, wifi, bluetooth 조회 후 반환
        """
        result: Dict[str, Optional[Network]] = {"ethernet": None, "wifi": None, "bluetooth": None}

        # 1) 전체 디바이스 상태 조회
        for dev, typ, state in _get_devices_status():
            # 2) 디바이스 상태가 connected가 아니면 스킵
            if state != "connected":
                continue

            t = typ.lower()

            # 3) 디바이스 타입에 따라 상세 정보 조회 후 Network 객체 생성
            if t in {"ethernet"}:
                result["ethernet"] = _build_network(dev, t) or result["ethernet"]
            elif t in {"wifi", "802-11-wireless"}:
                result["wifi"] = _build_network(dev, t) or result["wifi"]
            elif t in {"bluetooth", "bt"}:
                result["bluetooth"] = _build_network(dev, t) or result["bluetooth"]

        if ssid is not None:
            print("=============> wifi ssid : ", result["wifi"].ssid, ssid)
            if result["ethernet"] is not None and result["ethernet"].ssid == ssid:
                return result["ethernet"]
            elif result["wifi"] is not None and result["wifi"].ssid == ssid:
                return result["wifi"]
            elif result["bluetooth"] is not None and result["bluetooth"].ssid == ssid:
                return result["bluetooth"]
            else:
                return None

        return result

    async def get_ethernet(self) -> Optional[Network]:
        """
        [이더넷 조회]
        """
        return self.get_network()["ethernet"] or None

    async def get_wifi(self) -> Optional[Network]:
        """
        [와이파이 조회]
        get_network에서 조회한 데이터 + 신호강도를 위한 nmcli dev wifi list 조회 추가(시간 좀 걸림)
        """
        return self.get_network()["wifi"] or None

    async def get_bluetooth(self) -> Optional[Network]:
        """
        [블루투스 조회]
        """
        return self.get_network()["bluetooth"] or None

    def netmask_to_prefix(self, mask: str) -> int:
        # 255.255.255.0 -> 24
        return ipaddress.IPv4Network(f"0.0.0.0/{mask}").prefixlen

    async def set_network(self, model: NetworkModel) -> dict:
        """
        [네트워크 설정]
        """
        if model.dhcp is True:
            # nmcli device set <device> ipv4.method auto
            code, out, err = _run_cmd(["sudo","nmcli", "con", "modify", model.ssid, "ipv4.method", "auto"])
            if code != 0:
                rb_log.error(f"[network] setNetwork ERROR: {err}")
                if ("unknown connection" in err):
                    raise ServiceException(f"{model.ssid} 와이파이 네트워크를 찾을 수 없습니다.", 404)
                raise ServiceException(err, 500)

        else:
            dns_str = ",".join(model.dns)
            prefix = self.netmask_to_prefix(model.netmask)  # model.netmask = "255.255.255.0"
            addr_str = f"{model.address}/{prefix}"
            code, out, err = _run_cmd(["sudo","nmcli", "con", "modify", model.ssid, "ipv4.addresses", f"{addr_str}", "ipv4.gateway", model.gateway, "ipv4.dns", dns_str, "ipv4.method", "manual"])
            if code != 0:
                rb_log.error(f"[network] setNetwork ERROR: {err}")
                if ("unknown connection" in err):
                    raise ServiceException(f"{model.ssid} 와이파이 네트워크를 찾을 수 없습니다.", 404)
                raise ServiceException(err, 500)

        code, out, err = _run_cmd(["sudo","nmcli", "con", "up", model.ssid])
        if code != 0:
            rb_log.error(f"[network] setNetwork ERROR: {err}")
            raise ServiceException(err, 500)

        return await self.get_network(model.ssid)


    async def connect_wifi(self, model: NetworkModel) -> dict:
        """
        [와이파이 연결]
        """

        # 1) 와이파이 연결 시도 (비밀번호 있는 경우 포함)
        if model.password is not None and model.password != "":
            code, out, err = _run_cmd(["sudo","nmcli", "dev", "wifi", "connect", model.ssid, "password", model.password])
        else:
            code, out, err = _run_cmd(["sudo","nmcli", "dev", "wifi", "connect", model.ssid])

        # 2) 에러 처리
        if err != "" and err is not None:
            if ("No network" in err):
                raise ServiceException(f"{model.ssid} 와이파이 네트워크를 찾을 수 없습니다.", 404)
            if ("security" in err):
                raise ServiceException("비밀번호 형식이 올바르지 않습니다.", 400)
            rb_log.error(f"[network] connectWifi ERROR: {err}")
            raise ServiceException(err, 500)

        # 3) 비밀번호 누락 처리 (비밀번호 누락은 에러로 표기안됨)
        if ("Secrets were required" in out
            or "not provided" in out
            or "No secrets" in out
            or "property is missing" in out):
            rb_log.error(f"[network] connectWifi 비밀번호 에러 : {model.ssid}, {model.password}")
            raise ServiceException("비밀번호가 틀렸습니다.", 400)

        # 4) 연결 활성화
        code, out, err = _run_cmd(["sudo","nmcli", "con", "up", model.ssid])
        if code != 0:
            rb_log.error(f"[network] connectWifi ERROR: {err}")
            raise ServiceException(err, 500)

        # 5) 네트워크 정보 반환
        return await self.get_network(model.ssid)

    async def get_wifi_list(self, rescan: bool = False) -> list[WifiT]:
        """
        [와이파이 목록 조회]
        """
        # 1) 와이파이 최신 스캔
        if rescan:
            code, out, err = _run_cmd(["sudo","nmcli", "dev", "wifi", "rescan"])
            if code != 0:
                rb_log.error(f"[network] getWifiList ERROR: {err}")
                raise ServiceException(err, 500)

        # 2) 와이파이 목록 조회
        code, out, err = _run_cmd(["sudo","nmcli", "-t","-f", "IN-USE,SSID,SIGNAL,SECURITY,CHAN,RATE", "dev", "wifi", "list"])
        if code != 0:
            rb_log.error(f"[network] getWifiList ERROR: {err}")
            raise ServiceException(err, 500)

        result = []
        for line in out.splitlines():
            in_use, ssid, signal, security, chan, rate = line.split(":")
            if not ssid or ssid == "--":
                continue
            result.append(WifiT(
                inUse=in_use == "*",
                ssid=ssid,
                signal=int(signal),
                security=security,
                channel=int(chan),
                rate=rate
            ))

        return result
