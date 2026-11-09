# Message Schema 사용 방법 

이 레포지토리는 Git Subtree 방식으로 사용하는 message-schema를 관리합니다.
`schemas/` 디렉토리는 **직접 커밋 금지**이며, **반드시 제공된 Make 명령을 사용**해야 합니다.

___

## 1️⃣ Subtree를 프로젝트에 최초 1회 설정
### 1. message-schema를 Subtree로 추가
```sh
# 자신의 팀 프로젝트(repository) main 또는 develop 브랜치에서

git remote add message-schema https://github.com/RainbowRobotics/message-schema.git
git subtree add --prefix=schemas message-schema main --squash

git push origin <your-branch>
```

### 2. Makefile 설정
```Makefile
SCHEMA_DIR := schemas # 본인이 원하는 스키마 디렉토리명 (기본값: schemas)
SCHEMA_REMOTE := message-schema # 본인이 원하는 remote 명 (기본값: message-schema)

.PHONY: schema-update
schema-update:
	@bash "$(SCHEMA_DIR)/schema-update.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)

.PHONY: schema-sync
schema-sync:
	@bash "$(SCHEMA_DIR)/schema-sync.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)
```

### 3. Git 사용자 이메일 확인
```sh
git config user.email


# 없으면 설정:

git config user.email "your.name@company.com"
```

## 2️⃣ 기본 사용법
### 📤 Schema 변경사항 푸시

```sh
# schemas 수정했다면 (ex. vim schemas/nexus/v1/test.fbs)

# 변경사항 반영
make schema-update
```

#### 동작 요약

1. `schemas/` 변경사항을 **메인 레포에 자동 커밋 & push**

2. `message-schema` 레포에
`schema/from-<your-id>` **브랜치 생성 및 push**

3. `message-schema` 레포의 **GitHub Action이 PR을 자동 생성**

4. **PR은 자동 머지되지 않으며,**
message-schema 레포에서 **리뷰 후 수동 Apply** 되어야 main에 반영됨

📌 `make schema-update`는 **PR 생성까지만 자동화**합니다.<br />
`message-schema/main` 머지는 **사람이 직접 승인**합니다.

<br />

### 📥 message-schema 최신 변경사항 가져오기

```sh
make schema-sync
```

#### 동작 요약

1. 현재 작업 중 변경사항이 있으면 자동 stash
2. `message-schema` 레포의 `main` 브랜치 최신 상태만 pull
3. **PR이 이미 main에 머지된 schema만** `schemas/` subtree에 반영
4. stash 자동 복원

📌 아직 PR이 머지되지 않은
`schema/from-*` 브랜치의 변경사항은 **절대 포함되지 않습니다.**
<br />

## 3️⃣ 디렉토리 구조
```graphql
main-repo/
├── schemas/              # message-schema subtree
│   ├── schema-update.sh
│   ├── schema-sync.sh
│   └── nexus/...
├── src/
├── Makefile
└── README.md
```

## 4️⃣ 대표 시나리오
### 새로운 schema 추가 / 수정

```sh
vim schemas/nexus/v1/new_message.fbs
make schema-update
```

### 다른 팀원의 변경사항 받기
```sh
make schema-sync
```

### 충돌 발생 시

```sh
make schema-sync
# 충돌 발생

# 수동 해결 후
git add schemas/
git commit -m "Resolve schema conflicts"
git push origin <branch>
```

## 5️⃣ 반드시 지켜야 할 규칙 ⚠️
### ❌ 금지

- `schemas/`를 직접 커밋하지 말 것
- `git add .` 상태에서 `schema-update` 실행 금지
- `message-schema`를 별도로 클론해서 작업 금지

```sh
# ❌ 잘못된 방법
git add schemas/
git commit -m "Update schemas"

# ✅ 올바른 방법
make schema-update
```

### ✅ 권장

- 작업 시작 전 항상 동기화
    ```sh
    make schema-sync
    ```

- 개인 브랜치 사용
`schema/from-<your-id>` → `main` PR로 머지

## 6️⃣ 핵심 요약

- `schemas`는 **subtree**
- 직접 커밋 금지, make 명령만 사용
- `schema-update` = **PR 생성**
- `schema-sync` = **main 기준 동기화**
- **main 머지는 반드시 리뷰 후 수동 승인**