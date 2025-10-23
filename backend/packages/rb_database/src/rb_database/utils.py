from typing import Literal

from rb_utils.date import parse_date_with_time
from rb_utils.parser import to_iso

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


def make_check_include_query(
    key: str, values: list[str] | str | None, *, map: dict | None = None, query: dict
):
    if values is not None:
        # 리스트인 경우 각각 처리
        if isinstance(values, list | tuple):
            print("values", values, flush=True)
            vals = []
            for v in values:
                if isinstance(v, int):
                    vals.append(v)
                elif isinstance(v, str):
                    if v.isdigit():
                        vals.append(int(v))
                    elif map and v in map:
                        vals.append(map[v])
                    else:
                        vals.append(v)
            if vals:
                query[key] = {"$in": vals}
        else:
            # 단일 값
            if isinstance(values, int):
                query[key] = values
            elif isinstance(values, str):
                if values.isdigit():
                    query[key] = int(values)
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
