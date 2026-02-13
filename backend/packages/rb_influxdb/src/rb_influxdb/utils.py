"""
InfluxDB 유틸리티 함수
"""
from collections.abc import Mapping
from typing import Any

from influxdb_client.client.write.point import Point
from rb_utils.parser import t_to_dict


def flatbuffer_to_point(
    obj: Any,
    measurement: str,
    tags: dict[str, str] | None = None,
    prefix: str = "",
) -> Point:
    """
    FlatBuffer 객체를 InfluxDB Point로 변환

    Args:
        obj: FlatBuffer 객체 (StatusT 등)
        measurement: 측정값 이름
        tags: 태그 딕셔너리 (문자열만 가능)
        prefix: 필드명 접두사 (중첩된 객체 구분용)

    Returns:
        InfluxDB Point 객체
    """
    # FlatBuffer 객체를 딕셔너리로 변환
    obj_dict = t_to_dict(obj)

    # Point 생성
    point = Point(measurement)

    # 태그 추가
    if tags:
        for key, value in tags.items():
            if value is not None:
                point = point.tag(key, str(value))

    # 필드 추가 (재귀적으로 모든 필드를 추가)
    _add_fields_to_point(point, obj_dict, prefix)

    return point


def _add_fields_to_point(point: Point, data: Any, prefix: str = "") -> None:
    """
    딕셔너리 데이터를 Point의 필드로 추가 (재귀)

    Args:
        point: InfluxDB Point 객체
        data: 추가할 데이터 (딕셔너리, 리스트, 원시 타입)
        prefix: 필드명 접두사
    """
    if data is None:
        return

    # 원시 타입
    if isinstance(data, (bool, int, float, str)):
        field_name = prefix if prefix else "value"
        if isinstance(data, bool):
            point.field(field_name, data)
        elif isinstance(data, (int, float)):
            point.field(field_name, float(data))
        else:
            point.field(field_name, str(data))
        return

    # 딕셔너리
    if isinstance(data, Mapping):
        for key, value in data.items():
            # None 값은 스킵
            if value is None:
                continue

            # 필드명 생성 (접두사가 있으면 점으로 연결)
            field_name = f"{prefix}.{key}" if prefix else key

            # 중첩된 딕셔너리나 리스트는 재귀 호출
            if isinstance(value, (dict, list)):
                _add_fields_to_point(point, value, field_name)
            else:
                # 원시 타입은 직접 추가
                if isinstance(value, bool):
                    point.field(field_name, value)
                elif isinstance(value, (int, float)):
                    point.field(field_name, float(value))
                elif isinstance(value, str):
                    point.field(field_name, str(value))
        return

    # 리스트
    if isinstance(data, (list, tuple)):
        # 리스트는 인덱스와 함께 필드로 추가
        for idx, item in enumerate(data):
            field_name = f"{prefix}[{idx}]" if prefix else f"[{idx}]"
            if isinstance(item, (dict, list)):
                _add_fields_to_point(point, item, field_name)
            else:
                if isinstance(item, bool):
                    point.field(field_name, item)
                elif isinstance(item, (int, float)):
                    point.field(field_name, float(item))
                elif isinstance(item, str):
                    point.field(field_name, str(item))
        return
