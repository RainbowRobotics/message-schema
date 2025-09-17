from typing import Any

from utils.parser import t_to_dict


def ManipulateZenohResHelper(obj: Any):
    dict_obj = t_to_dict(obj)

    if dict_obj.hasAttr("returnValue"):
        dict_obj.set("return_value", dict_obj.get("returnValue"))
        del dict_obj["returnValue"]

    raise ValueError("obj is not a dict")
