FROM eclipse/zenoh:latest

COPY zenoh.json5 /etc/zenoh/config.json5

# 포트 노출
EXPOSE 7447

# Health check 추가
HEALTHCHECK --interval=10s --timeout=5s --retries=3 \
  CMD sh -c 'netstat -tuln | grep :7447 || exit 1'

# Zenoh router 실행 (entrypoint가 이미 zenohd임)
CMD ["-c", "/etc/zenoh/config.json5"]