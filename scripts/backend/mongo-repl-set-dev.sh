
# ========= MongoDB 설정 =========
MONGO_CONTAINER_NAME="rrs-mongo-dev"
MONGO_VOLUME_NAME="rrs-mongo-data"
MONGO_RS_NAME="rs0"

if ! docker volume inspect "$MONGO_VOLUME_NAME" &>/dev/null; then
  echo "🔄 MongoDB 데이터 볼륨 생성 중..."
  docker volume create "$MONGO_VOLUME_NAME"
fi

if ! docker ps -a --format '{{.Names}}' | grep -Fxq "$MONGO_CONTAINER_NAME"; then
  echo "🔄 MongoDB 컨테이너 생성 중..."
  docker run -d \
    --name "$MONGO_CONTAINER_NAME" \
    --network host \
    --restart unless-stopped \
    -v "$MONGO_VOLUME_NAME:/data/db" \
    mongo:7 \
    --replSet "$MONGO_RS_NAME" \
    --bind_ip_all >/dev/null

else
  # 컨테이너 존재 → 실행 상태 확인
  STATUS=$(docker inspect -f '{{.State.Status}}' "$MONGO_CONTAINER_NAME" 2>/dev/null || echo "unknown")

  if [ "$STATUS" = "exited" ] || [ "$STATUS" = "created" ]; then
    echo "▶️  MongoDB 컨테이너가 정지되어 있습니다. 시작합니다..."
    docker start "$MONGO_CONTAINER_NAME" >/dev/null
  fi
fi

# ========= MongoDB 레플리카셋 확인 및 초기화 =========
for i in {1..30}; do
  if docker exec "$MONGO_CONTAINER_NAME" mongosh --quiet --eval "db.adminCommand('ping').ok" >/dev/null 2>&1; then
    break
  fi
  echo "⏳ MongoDB 대기 중... ($i/30)"
  sleep 1
done

echo "🔍 MongoDB 레플리카셋 상태 확인 중..."
REPL_STATUS=$(docker exec "$MONGO_CONTAINER_NAME" mongosh --quiet --eval "
  try {
    const s = rs.status();
    if (s.ok) { print('OK'); }
  } catch (e) { print(e.codeName || e.message); }
")

if [[ "$REPL_STATUS" == *"NotYetInitialized"* ]]; then
  echo "⚙️  레플리카셋이 초기화되지 않았습니다. rs.initiate() 실행 중..."
  docker exec "$MONGO_CONTAINER_NAME" mongosh --quiet --eval "
    rs.initiate({
      _id: '$MONGO_RS_NAME',
      members: [{ _id: 0, host: 'rrs-mongo-dev:27017' }]
    });
  "
  echo "✅ Replica set initialized successfully."

elif [[ "$REPL_STATUS" == *"ok"* ]] || [[ "$REPL_STATUS" == *"OK"* ]]; then
  echo "✅ 이미 레플리카셋 활성화 상태입니다."

elif [[ "$REPL_STATUS" == *"not started with replication enabled"* ]] \
   || [[ "$REPL_STATUS" == *"ReplicationNotEnabled"* ]] \
   || [[ "$REPL_STATUS" == *"NoReplicationEnabled"* ]]; then
  echo "⚠️  mongod가 replSet 없이 시작되었습니다. 컨테이너를 재생성합니다..."

  # 기존 컨테이너 정지 및 삭제
  docker stop "$MONGO_CONTAINER_NAME" >/dev/null || true
  docker rm "$MONGO_CONTAINER_NAME" >/dev/null || true

  # local DB 초기화(필요 시, 기존 데이터 유지하면서 replSet 설정)
  docker run --rm -v "$MONGO_VOLUME_NAME:/data/db" mongo:7 bash -c "rm -rf /data/db/local/*"

  # replSet 옵션을 준 새 컨테이너 생성
  docker run -d \
    --name "$MONGO_CONTAINER_NAME" \
    --network host \
    --restart unless-stopped \
    -v "$MONGO_VOLUME_NAME:/data/db" \
    mongo:7 \
    --replSet "$MONGO_RS_NAME" \
    --bind_ip_all >/dev/null

  # 다시 기동 대기 후 rs.initiate
  for i in {1..30}; do
    if docker exec "$MONGO_CONTAINER_NAME" mongosh --quiet --eval "db.adminCommand('ping').ok" >/dev/null 2>&1; then
      break
    fi
    echo "⏳ MongoDB 재기동 대기 중... ($i/30)"
    sleep 1
  done

  docker exec "$MONGO_CONTAINER_NAME" mongosh --quiet --eval "
    rs.initiate({
      _id: '$MONGO_RS_NAME',
      members: [{ _id: 0, host: 'rrs-mongo-dev:27017' }]
    });
  "
  echo "✅ 기존 standalone DB를 replSet 모드 컨테이너로 전환 완료."

else
  echo "⚠️  알 수 없는 레플리카셋 상태입니다: $REPL_STATUS"
fi


docker stop rrs-mongo-dev
