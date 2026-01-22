SHELL := /bin/bash ##항상 bash로 일관성
.ONESHELL: ##한 타겟의 여러 줄을 한 셸에서 실행 (cd 유지 등)
.SHELLFLAGS := -eu -o pipefail -c ##안전 모드 (set -euo pipefail)
MAKEFLAGS += --no-builtin-rules ##make의 쓸데없는 자동 규칙 차단, 오직 내가 정의한 규칙만 실행

include backend/backend.mk

.DEFAULT_GOAL := help

SCHEMA_DIR := schemas
SCHEMA_REMOTE := rb_schemas
SCHEMA_REMOTE_URL := https://github.com/RainbowRobotics/message-schema.git

.PHONY: help
help: ## 가능한 타겟 설명 출력
	@awk 'BEGIN {FS = ":.*?## "}; /^[a-zA-Z0-9_.-]+:.*?## / {printf "  \033[36m%-28s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | sort

.PHONY: schema-subtree-init
schema-subtree-init: ## schema subtree 최초 1회 초기화
	@[ ! -d "$(SCHEMA_DIR)" ] || (echo "❌ $(SCHEMA_DIR) 이라는 디렉토리가 이미 존재합니다. 지워주시고 커밋/푸시 후 진행해주세요!"; exit 1)
	@git remote get-url $(SCHEMA_REMOTE) >/dev/null 2>&1 \
	  || git remote add $(SCHEMA_REMOTE) $(SCHEMA_REMOTE_URL)
	@git subtree add --prefix=$(SCHEMA_DIR) $(SCHEMA_REMOTE) main --squash

.PHONY: schema-update
schema-update: ## schemas 변경사항을 현재 레포와 서브레포로 push 그리고 자동 PR 진행
	@bash "$(SCHEMA_DIR)/schema-update.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)

.PHONY: schema-sync
schema-sync: ## message-schema의 main 브랜치 내용을 현재 SCHEMA_DIR에 그대로 가져오기 (변경 사항이 있다면 진짜 덮어씌울건지 물어봄)
	@bash "$(SCHEMA_DIR)/schema-sync.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)
