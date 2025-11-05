# Fbs 업데이트 하는 방법

```bash
flatc --cpp --filename-suffix "_generated" --filename-ext "h" --gen-object-api -o "rb/v1" *.fbs
```


```bash
cd build
./examples/session_pub ../examples/config.json5
./examples/session_sub ../examples/config.json5
./examples/session_serve ../examples/config.json5
./examples/session_call ../examples/config.json5
```