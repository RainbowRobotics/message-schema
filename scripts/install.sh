tar -xzf build

mkdir -p lib

for f in lib-part-*.tar.gz; do
  echo "📦 $f 해제 중..."
  tar -xzf "$f" -C lib
done