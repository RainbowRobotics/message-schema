from typing import (
    Any,
    Literal,
)

from rb_utils.date import (
    parse_date_with_time,
)
from rb_utils.parser import (
    to_iso,
)

RangeDict = dict[Literal["from", "to"], str | int]


def make_check_search_text_query(
    key: str, searchText: str | None, *, useTextIndex: bool = False, query: dict
):
    if searchText is not None:
        if useTextIndex:
            return {"$text": {"$search": searchText}}
        else:
            query[key] = {"$regex": searchText, "$options": "i"}
    return query


def make_check_include_query(key: str, values: Any, *, list_map: dict | None = None, query: dict):
    if values is None:
        return query

    if isinstance(values, list | tuple):
        vals = []

        for v in values:
            # 숫자는 그대로
            if isinstance(v, int | float):
                vals.append(v)
                continue

            if isinstance(v, str):
                s = v.strip()

                # 숫자 문자열 → int
                if s.isdigit():
                    vals.append(int(s))
                    continue

                # 매핑 있으면 매핑값
                if list_map and s in list_map:
                    vals.append(list_map[s])
                    continue

                # 숫자 변환 안 되면 문자열 그대로
                vals.append(s)
                continue

            # 그 외 타입도 그대로
            vals.append(v)

        if vals:
            query[key] = {"$in": vals}

        return query

    # 단일 값
    if isinstance(values, int | float):
        query[key] = values
    elif isinstance(values, str):
        s = values.strip()
        if s.isdigit():
            query[key] = int(s)
        elif list_map and s in list_map:
            query[key] = list_map[s]
        else:
            query[key] = s
    else:
        query[key] = values

    return query



def make_check_date_range_query(key: str, range: RangeDict, *, query: dict):
    if not isinstance(range, dict):
        raise TypeError("values must be a dict with keys 'from' and 'to'")

    if "from" not in range or "to" not in range:
        raise ValueError("values must contain both 'from' and 'to' keys")

    start = range["from"]
    end = range["to"]

    if not isinstance(start, str | int):
        raise TypeError("'from' must be str or int")
    if not isinstance(end, str | int):
        raise TypeError("'to' must be str or int")

    query.setdefault(key, {})
    q = query[key]

    q["$gte"] = to_iso(parse_date_with_time("start", start))
    q["$lte"] = to_iso(parse_date_with_time("end", end))

    query[key] = q

    return query
