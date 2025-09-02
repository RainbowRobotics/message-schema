def snake_to_camel(s: str) -> str:
    parts = s.split("_")
    return "".join(p[:1].upper() + p[1:] for p in parts if p)



def camel_to_snake(s: str) -> str:
    return "".join("_" + word.lower() if word.isupper() else word for word in s).lstrip("_")

