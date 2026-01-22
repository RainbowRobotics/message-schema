"""
[Email API]
"""



from fastapi import APIRouter
from rb_modules.log import RBLog

from .network_module import (
    NetworkService,
)
from .network_schema import (
    Response_Network_GetNetwork,
    Request_Network_ConnectWifi,
    Request_Network_GetWifiList,
    Request_Network_SetPD,
    Response_Network_ConnectWifi,
    Response_Network_GetWifiList,
    Response_Network_SetPD,
)

network_service = NetworkService()
network_router = APIRouter(
    tags=["Network"],
    prefix="/network"
)
rb_log = RBLog()

@network_router.get(
    "",
    summary="현재 네트워크 조회(이더넷,와이파이,블루투스)",
    description="""
현재 네트워크 연결 상태를 조회합니다.

## 📌 기능 설명
- 이더넷, 와이파이, 블루투스 디바이스에 연결된 네트워크가 있다면 그 정보를 반환합니다.
- 반환하는 정보는 ssid, address 의 값을 포함합니다.

## 📌 응답 바디(JSON)

| 필드명 | 타입 | 설명 | 예시 |
|-|-|-|-|
| ethernet | Network | 이더넷 연결된 네트워크 정보 |  |
| wifi | Network | 와이파이 연결된 네트워크 정보 |  |
| bluetooth | Network | 블루투스 연결된 네트워크 정보 |  |

## 📌 Network 모델(JSON)

| 필드명 | 타입 | 설명 | 예시 |
|-|-|-|-|
| device | string | 디바이스 이름 | eth0 |
| dhcp | bool | DHCP 사용 여부 | true |
| dns | list[string] | DNS 서버 목록 | ["8.8.8.8", "8.8.4.4"] |
| ssid | string | 와이파이 접속 이름 | mobile_team |
| address | string | IP 주소 | 192.168.1.100 |
| netmask | string | 네트워크 마스크 | 255.255.255.0 |
| gateway | string | 게이트웨이 주소 | 192.168.1.1 |
| signal | int | 신호 강도. wifi만 해당 | 100 |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 요청한 명령이 지원하지 않는 명령일 때
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
    """
)
async def get_network():
    """
    [현재 네트워크 조회(이더넷,와이파이,블루투스)]
    """
    return await network_service.get_network()


# @network_router.put(
#     "",
#     summary="네트워크 설정",
#     description="""
# 네트워크 설정을 변경합니다.

# ## 📌 기능 설명
# - 와이파이 접속 이름, DHCP 사용 여부, IP 주소, 네트워크 마스크, 게이트웨이 주소, DNS 서버 목록을 입력으로 받아 네트워크 설정을 변경합니다.
# - DHCP 사용 여부를 true로 설정하면 IP 주소, 네트워크 마스크, 게이트웨이 주소, DNS 서버 목록을 무시합니다.
# - 변경할 네트워크의 SSID값이 반드시 필요합니다.

# ## 📌 요청 바디(JSON)

# | 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
# |-|-|-|-|
# | ssid | string | ✅ | - | 와이파이 접속 이름 | mobile_team |
# | dhcp | bool | ✅ | - | DHCP 사용 여부 | true |
# | address | string | - | - | IP 주소 | 192.168.1.100 |
# | gateway | string | - | - | 게이트웨이 주소 | 192.168.1.1 |
# | netmask | string | - | - | 네트워크 마스크 | 255.255.255.0 |
# | dns | list[string] | - | - | DNS 서버 목록 | ["8.8.8.8", "8.8.4.4"] |

# ## 📌 응답 바디(JSON)

# | 필드명 | 타입 | 설명 | 예시 |
# |-|-|-|-|
# | ssid | string | 와이파이 접속 이름 | mobile_team |
# | dhcp | bool | DHCP 사용 여부 | true |
# | address | string | IP 주소 | 192.168.1.100 |
# | gateway | string | 게이트웨이 주소 | 192.168.1.1 |
# | netmask | string | 네트워크 마스크 | 255.255.255.0 |
# | dns | list[string] | DNS 서버 목록 | ["8.8.8.8", "8.8.4.4"] |
# | result | string | 결과 | success |
# | message | string | 메시지 | 네트워크 설정이 완료되었습니다. |

# ## ⚠️ 에러 케이스
# ### **403** INVALID_ARGUMENT
#   - 요청한 명령이 지원하지 않는 명령일 때
#   - 파라메터가 없거나 잘못된 값일 때
# ### **409** CONFLICT
#   - 요청한 명령을 수행할 수 없을 때
# ### **500** INTERNAL_SERVER_ERROR
#   - DB관련 에러 등 서버 내부적인 에러
#     """
# )
# async def set_network(request: Request_Network_SetPD) -> Response_Network_SetPD:
#     return await network_service.set_network(request)

# @network_router.post(
#     "wifi",
#     summary="와이파이 접속",
#     description="""
# 와이파이 접속을 시도합니다.

# ## 📌 기능 설명
# - 와이파이 접속 이름을 입력으로 받아 와이파이 접속을 시도합니다.
# - 비밀번호는 선택사항이며 해당 와이파이에 보안옵션이 있을 경우 비밀번호를 입력해야 합니다.

# ## 📌 요청 바디(JSON)

# | 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
# |-|-|-|-|
# | ssid | string | ✅ | - | 와이파이 접속 이름 | mobile_team |
# | password | string | - | - | 비밀번호 | 1234 |

# ## 📌 응답 바디(JSON)

# | 필드명 | 타입 | 설명 | 예시 |
# |-|-|-|-|
# | ssid | string | 와이파이 접속 이름 | mobile_team |
# | dhcp | bool | DHCP 사용 여부 | true |
# | address | string | IP 주소 | 192.168.1.100 |
# | gateway | string | 게이트웨이 주소 | 192.168.1.1 |
# | netmask | string | 네트워크 마스크 | 255.255.255.0 |
# | dns | list[string] | DNS 서버 목록 | ["8.8.8.8", "8.8.4.4"] |
# | result | string | 결과 | success |
# | message | string | 메시지 | 네트워크 설정이 완료되었습니다. |

# ## ⚠️ 에러 케이스
# ### **403** INVALID_ARGUMENT
#   - 요청한 명령이 지원하지 않는 명령일 때
#   - 파라메터가 없거나 잘못된 값일 때
# ### **409** CONFLICT
#   - 요청한 명령을 수행할 수 없을 때
# ### **500** INTERNAL_SERVER_ERROR
#   - DB관련 에러 등 서버 내부적인 에러
#     """
# )
# async def connect_wifi(request: Request_Network_ConnectWifi) -> Response_Network_ConnectWifi:
#     return await network_service.connect_wifi(request)

# @network_router.get(
#     "wifi/list",
#     summary="와이파이 목록 조회",
#     description="""
# 와이파이 목록을 조회합니다.

# ## 📌 기능 설명
# - 와이파이 목록을 조회합니다.
# - rescan 파라메터를 true로 설정하면 와이파이 목록을 다시 조회합니다.

# ## 📌 요청 쿼리

# | 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
# |-|-|-|-|-|-|
# | rescan | bool | - | - | 와이파이 목록을 다시 조회할지 여부 | true |

# ## 📌 응답 바디(JSON)

# | 필드명 | 타입 | 설명 | 예시 |
# |-|-|-|-|
# | list | list[Wifi] | 와이파이 목록 | [{"in_use": true, "ssid": "mobile_team", "signal": 100, "security": "WPA2", "channel": 1, "rate": "100Mbps"}] |
# | result | string | 결과 | success |
# | message | string | 메시지 | 와이파이 목록 조회가 완료되었습니다. |

# ## 📌 Wifi 모델(JSON)

# | 필드명 | 타입 | 설명 | 예시 |
# |-|-|-|-|
# | in_use | bool | 사용 여부 | true |
# | ssid | string | 와이파이 접속 이름 | mobile_team |
# | signal | int | 신호 강도 | 100 |
# | security | string | 보안 옵션 | WPA2 |
# | channel | int | 채널 | 1 |
# | rate | string | 속도 | 100Mbps |

# ## ⚠️ 에러 케이스
# ### **403** INVALID_ARGUMENT
#   - 요청한 명령이 지원하지 않는 명령일 때
#   - 파라메터가 없거나 잘못된 값일 때
# ### **409** CONFLICT
#   - 요청한 명령을 수행할 수 없을 때
# ### **500** INTERNAL_SERVER_ERROR
#   - DB관련 에러 등 서버 내부적인 에러
#     """
# )
# async def get_wifi_list(rescan: bool = False) -> Response_Network_GetWifiList:
#     return await network_service.get_wifi_list(rescan)
