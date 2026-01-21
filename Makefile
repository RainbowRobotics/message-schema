SHELL := /bin/bash ##항상 bash로 일관성
.ONESHELL: ##한 타겟의 여러 줄을 한 셸에서 실행 (cd 유지 등)
.SHELLFLAGS := -eu -o pipefail -c ##안전 모드 (set -euo pipefail)
MAKEFLAGS += --no-builtin-rules ##make의 쓸데없는 자동 규칙 차단, 오직 내가 정의한 규칙만 실행

include backend/backend.mk

.DEFAULT_GOAL := help

SCHEMA_DIR := schemas
SCHEMA_REMOTE := rb_schemas

.PHONY: help
help: ## 가능한 타겟 설명 출력
	@awk 'BEGIN {FS = ":.*?## "}; /^[a-zA-Z0-9_.-]+:.*?## / {printf "  \033[36m%-28s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | sort

.PHONY: schema-update
schema-update: ## schema 변경 사항을 message-schema 레포로 push
	@bash "$(SCHEMA_DIR)/schema-update.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)
.PHONY: schema-sync
schema-sync: ## $(SCHEMA_DIR) 레포의 최신상태를 가져오기ㄷㄷ
	@bash "$(SCHEMA_DIR)/schema-sync.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)
