# SLAMNAV Zenoh Topics Reference

이 문서는 SLAMNAV Zenoh 통신의 토픽 구조를 정리한 것입니다.

---

## 토픽 네이밍 컨벤션

```
slamnav/{robot_id}/{domain}/{action}/{sub_action}
```

- `robot_id`: 로봇 식별자
- `domain`: 기능 도메인 (control, localization, map, move, setting, status, update)
- `action`: 동작 유형 (get, set, request, response, state)
- `sub_action`: 세부 동작

---

## 1. Control 토픽

### Safety Field
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/get/safety_field` | Request | `Request_Get_Safety_Field` |
| `slamnav/{id}/control/get/safety_field/response` | Response | `Response_Get_Safety_Field` |
| `slamnav/{id}/control/set/safety_field` | Request | `Request_Set_Safety_Field` |
| `slamnav/{id}/control/set/safety_field/response` | Response | `Response_Set_Safety_Field` |

### Safety Flag
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/get/safety_flag` | Request | `Request_Get_Safety_Flag` |
| `slamnav/{id}/control/get/safety_flag/response` | Response | `Response_Get_Safety_Flag` |
| `slamnav/{id}/control/set/safety_flag` | Request | `Request_Set_Safety_Flag` |
| `slamnav/{id}/control/set/safety_flag/response` | Response | `Response_Set_Safety_Flag` |

### Safety IO
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/get/safety_io` | Request | `Request_Get_Safety_Io` |
| `slamnav/{id}/control/get/safety_io/response` | Response | `Response_Get_Safety_Io` |
| `slamnav/{id}/control/set/safety_io` | Request | `Request_Set_Safety_Io` |
| `slamnav/{id}/control/set/safety_io/response` | Response | `Response_Set_Safety_Io` |

### Dock
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/dock` | Request | `Request_Dock` |
| `slamnav/{id}/control/dock/response` | Response | `Response_Dock` |
| `slamnav/{id}/control/dock/state` | Publish | `State_Change_Dock` |
| `slamnav/{id}/control/charge_trigger` | Request | `Request_Charge_Trigger` |
| `slamnav/{id}/control/charge_trigger/response` | Response | `Response_Charge_Trigger` |

### Obstacle Box
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/get/obs_box` | Request | `Request_Get_Obs_Box` |
| `slamnav/{id}/control/get/obs_box/response` | Response | `Response_Get_Obs_Box` |
| `slamnav/{id}/control/set/obs_box` | Request | `Request_Set_Obs_Box` |
| `slamnav/{id}/control/set/obs_box/response` | Response | `Response_Set_Obs_Box` |

### LED
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/led` | Request | `Request_Led` |
| `slamnav/{id}/control/led/response` | Response | `Response_Led` |

### Motor
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/motor` | Request | `Request_Motor` |
| `slamnav/{id}/control/motor/response` | Response | `Response_Motor` |

### Jog
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/jog` | Request | `Request_Jog` |
| `slamnav/{id}/control/jog/response` | Response | `Response_Jog` |

### Sensor Control
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/sensor` | Request | `Request_Sensor` |
| `slamnav/{id}/control/sensor/response` | Response | `Response_Sensor` |

### Path Control
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/path` | Request | `Request_Path` |
| `slamnav/{id}/control/path/response` | Response | `Response_Path` |

### Detect
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/control/detect` | Request | `Request_Detect` |
| `slamnav/{id}/control/detect/response` | Response | `Response_Detect` |

---

## 2. Localization 토픽

| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/localization/init` | Request | `Request_Localization_Init` |
| `slamnav/{id}/localization/init/response` | Response | `Response_Localization_Init` |
| `slamnav/{id}/localization/random_init` | Request | `Request_Localization_RandomInit` |
| `slamnav/{id}/localization/random_init/response` | Response | `Response_Localization_RandomInit` |
| `slamnav/{id}/localization/auto_init` | Request | `Request_Localization_AutoInit` |
| `slamnav/{id}/localization/auto_init/response` | Response | `Response_Localization_AutoInit` |
| `slamnav/{id}/localization/semi_auto_init` | Request | `Request_Localization_SemiAutoInit` |
| `slamnav/{id}/localization/semi_auto_init/response` | Response | `Response_Localization_SemiAutoInit` |
| `slamnav/{id}/localization/start` | Request | `Request_Localization_Start` |
| `slamnav/{id}/localization/start/response` | Response | `Response_Localization_Start` |
| `slamnav/{id}/localization/stop` | Request | `Request_Localization_Stop` |
| `slamnav/{id}/localization/stop/response` | Response | `Response_Localization_Stop` |

---

## 3. Map 토픽

### Map 관리
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/map/list` | Request | `Request_Map_List` |
| `slamnav/{id}/map/list/response` | Response | `Response_Map_List` |
| `slamnav/{id}/map/delete` | Request | `Request_Map_Delete` |
| `slamnav/{id}/map/delete/response` | Response | `Response_Map_Delete` |
| `slamnav/{id}/map/current` | Request | `Request_Map_Current` |
| `slamnav/{id}/map/current/response` | Response | `Response_Map_Current` |
| `slamnav/{id}/map/load` | Request | `Request_Map_Load` |
| `slamnav/{id}/map/load/response` | Response | `Response_Map_Load` |

### Map File/Cloud
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/map/get/file` | Request | `Request_Get_Map_File` |
| `slamnav/{id}/map/get/file/response` | Response | `Response_Get_Map_File` |
| `slamnav/{id}/map/get/cloud` | Request | `Request_Get_Map_Cloud` |
| `slamnav/{id}/map/get/cloud/response` | Response | `Response_Get_Map_Cloud` |
| `slamnav/{id}/map/set/cloud` | Request | `Request_Set_Map_Cloud` |
| `slamnav/{id}/map/set/cloud/response` | Response | `Response_Set_Map_Cloud` |

### Topology
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/map/get/topology` | Request | `Request_Get_Map_Topology` |
| `slamnav/{id}/map/get/topology/response` | Response | `Response_Get_Map_Topology` |
| `slamnav/{id}/map/set/topology` | Request | `Request_Set_Map_Topology` |
| `slamnav/{id}/map/set/topology/response` | Response | `Response_Set_Map_Topology` |

### Mapping
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/mapping/start` | Request | `Request_Mapping_Start` |
| `slamnav/{id}/mapping/start/response` | Response | `Response_Mapping_Start` |
| `slamnav/{id}/mapping/stop` | Request | `Request_Mapping_Stop` |
| `slamnav/{id}/mapping/stop/response` | Response | `Response_Mapping_Stop` |
| `slamnav/{id}/mapping/save` | Request | `Request_Mapping_Save` |
| `slamnav/{id}/mapping/save/response` | Response | `Response_Mapping_Save` |

---

## 4. Move 토픽

### 이동 명령
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/move/goal` | Request | `Request_Move_Goal` |
| `slamnav/{id}/move/goal/response` | Response | `Response_Move_Goal` |
| `slamnav/{id}/move/target` | Request | `Request_Move_Target` |
| `slamnav/{id}/move/target/response` | Response | `Response_Move_Target` |
| `slamnav/{id}/move/jog` | Publish | `Move_Jog` |

### 이동 제어
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/move/stop` | Request | `Request_Move_Stop` |
| `slamnav/{id}/move/stop/response` | Response | `Response_Move_Stop` |
| `slamnav/{id}/move/pause` | Request | `Request_Move_Pause` |
| `slamnav/{id}/move/pause/response` | Response | `Response_Move_Pause` |
| `slamnav/{id}/move/resume` | Request | `Request_Move_Resume` |
| `slamnav/{id}/move/resume/response` | Response | `Response_Move_Resume` |

### 특수 이동
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/move/xlinear` | Request | `Request_Move_XLinear` |
| `slamnav/{id}/move/xlinear/response` | Response | `Response_Move_XLinear` |
| `slamnav/{id}/move/circular` | Request | `Request_Move_Circular` |
| `slamnav/{id}/move/circular/response` | Response | `Response_Move_Circular` |
| `slamnav/{id}/move/rotate` | Request | `Request_Move_Rotate` |
| `slamnav/{id}/move/rotate/response` | Response | `Response_Move_Rotate` |

### 상태 변경
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/move/state` | Publish | `State_Change_Move` |

---

## 5. Setting 토픽

### Robot Type
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/setting/get/robot_type` | Request | `Request_Get_Robot_Type` |
| `slamnav/{id}/setting/get/robot_type/response` | Response | `Response_Get_Robot_Type` |
| `slamnav/{id}/setting/set/robot_type` | Request | `Request_Set_Robot_Type` |
| `slamnav/{id}/setting/set/robot_type/response` | Response | `Response_Set_Robot_Type` |

### PDU Param
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/setting/get/pdu_param` | Request | `Request_Get_Pdu_Param` |
| `slamnav/{id}/setting/get/pdu_param/response` | Response | `Response_Get_Pdu_Param` |
| `slamnav/{id}/setting/set/pdu_param` | Request | `Request_Set_Pdu_Param` |
| `slamnav/{id}/setting/set/pdu_param/response` | Response | `Response_Set_Pdu_Param` |

### Drive Param
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/setting/get/drive_param` | Request | `Request_Get_Drive_Param` |
| `slamnav/{id}/setting/get/drive_param/response` | Response | `Response_Get_Drive_Param` |

### Sensor Index
| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/setting/get/sensor_index` | Request | `Request_Get_Sensor_Index` |
| `slamnav/{id}/setting/get/sensor_index/response` | Response | `Response_Get_Sensor_Index` |
| `slamnav/{id}/setting/set/sensor_index` | Request | `Request_Set_Sensor_Index` |
| `slamnav/{id}/setting/set/sensor_index/response` | Response | `Response_Set_Sensor_Index` |
| `slamnav/{id}/setting/set/sensor_on` | Request | `Request_Set_Sensor_On` |
| `slamnav/{id}/setting/set/sensor_on/response` | Response | `Response_Set_Sensor_On` |
| `slamnav/{id}/setting/get/sensor_off` | Request | `Request_Get_Sensor_Off` |
| `slamnav/{id}/setting/get/sensor_off/response` | Response | `Response_Get_Sensor_Off` |

---

## 6. Status 토픽

| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/status` | Publish | `Status` |
| `slamnav/{id}/status/move` | Publish | `Move_Status` |

---

## 7. Sensor Data 토픽 (Socket)

| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/sensor/lidar2d` | Publish | `Lidar_2D` |
| `slamnav/{id}/sensor/lidar3d` | Publish | `Lidar_3D` |
| `slamnav/{id}/sensor/mapping_cloud` | Publish | `Mapping_Cloud` |
| `slamnav/{id}/path/global` | Publish | `Global_Path` |
| `slamnav/{id}/path/local` | Publish | `Local_Path` |

---

## 8. Multi Robot 토픽

| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/multi/path` | Request | `Request_Path` |
| `slamnav/{id}/multi/path/response` | Response | `Response_Path` |
| `slamnav/{id}/multi/vobs` | Request | `Request_Vobs` |
| `slamnav/{id}/multi/vobs/response` | Response | `Response_Vobs` |

---

## 9. Update 토픽

| 토픽 | 방향 | 메시지 타입 |
|------|------|------------|
| `slamnav/{id}/update` | Request | `Request_Update` |
| `slamnav/{id}/update/response` | Response | `Response_Update` |
| `slamnav/{id}/update/current_version` | Request | `Request_Current_Version` |
| `slamnav/{id}/update/current_version/response` | Response | `Response_Current_Version` |

---

## 통신 패턴

### 1. Request/Response 패턴
```
Client                           Server
  |                                |
  |------ Request_xxx ----------->|
  |                                |
  |<----- Response_xxx -----------|
  |                                |
```

### 2. Publish/Subscribe 패턴 (상태 스트리밍)
```
Publisher                      Subscriber(s)
  |                                |
  |------ Status/Move_Status ---->|
  |------ Lidar_2D/3D ----------->|
  |------ Global/Local_Path ----->|
  |                                |
```

### 3. State Change 패턴 (이벤트 알림)
```
Server                         Client(s)
  |                                |
  |------ State_Change_xxx ------>|
  |                                |
```
