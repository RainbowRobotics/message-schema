```bash
flatc --cpp -o rb_ipc/comm_generated rb_ipc/comm_fbs/manipulate/v1/*.fbs
flatc --cpp --filename-suffix "_generated" --filename-ext "h" --gen-object-api -o "rb/v1" *.fbs
flatc --cpp --filename-suffix "_generated" --filename-ext "h" --gen-object-api -o "rb_ipc/comm_generated" rb_ipc/comm_fbs/manipulate/v1/*.fbs

flatc --cpp --filename-suffix "_generated" --filename-ext "h" --gen-object-api -o "rb_ipc/comm_generated/manipulate/v1" rb_ipc/comm_fbs/manipulate/v1/*.fbs
flatc --cpp --filename-suffix "_generated" --filename-ext "h" --gen-object-api -o "rb_ipc/comm_generated/nexus/v1" rb_ipc/comm_fbs/nexus/v1/*.fbs
pmap -x 2504110
```

echo -e "call_move_j(0,0,0,90,0,90,0,0.4,0.4)" | nc 127.0.0.1 5000
echo -e "call_joint_brake(-1, 1)" | nc 127.0.0.1 5000
echo -e "call_joint_encoder_zero(6)" | nc 127.0.0.1 5000
---

KJ

- Code format 은 통일하는 것이 좋을 것 같습니다. (clang-format을 활용하시면 됩니다.)
- Docker를 활용한 개발환경 구축
  - https://learn.microsoft.com/ko-kr/training/modules/use-docker-container-dev-env-vs-code/3-use-as-development-environment
  - VSCode > F1 > Dev Containers: Rebuild and Reopen in Container