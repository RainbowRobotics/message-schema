SHELL := /bin/bash ##항상 bash로 일관성
.ONESHELL: ##한 타겟의 여러 줄을 한 셸에서 실행 (cd 유지 등)
.SHELLFLAGS := -eu -o pipefail -c ##안전 모드 (set -euo pipefail)
MAKEFLAGS += --no-builtin-rules ##make의 쓸데없는 자동 규칙 차단, 오직 내가 정의한 규칙만 실행

include backend/backend.mk

.DEFAULT_GOAL := help

SCHEMA_DIR := schemas
SCHEMA_REMOTE := rb_schemas
SCHEMA_REMOTE_URL := https://github.com/RainbowRobotics/message-schema

.PHONY: help
help: ## 가능한 타겟 설명 출력
	@awk 'BEGIN {FS = ":.*?## "}; /^[a-zA-Z0-9_.-]+:.*?## / {printf "  \033[36m%-28s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | sort

.PHONY: schema-update
schema-update: ## schema 변경 사항을 message-schema 레포로 push
	@set -euo pipefail; \
	BR="schema/from-$$(git config --get user.email | sed 's/@.*//' | tr -cd '[:alnum:]')"; \
	if [ -z "$$BR" ] || [ "$$BR" = "schema/from-" ]; then \
		echo "Error: Cannot determine branch name. Check git user.email"; \
		exit 1; \
	fi; \
	echo "schema branch => $$BR"; \
	\
	# remote 설정 및 fetch
	git remote get-url "$(SCHEMA_REMOTE)" >/dev/null 2>&1 || git remote add "$(SCHEMA_REMOTE)" "$(SCHEMA_REMOTE_URL)"; \
	git fetch "$(SCHEMA_REMOTE)" || true; \
	\
	# 로컬 schemas 디렉토리 트리 해시
	LOCAL_TREE=$$(git rev-parse "HEAD:$(SCHEMA_DIR)"); \
	echo "local  tree => $$LOCAL_TREE"; \
	\
	# 원격 브랜치 존재 여부 확인
	REMOTE_REF="refs/remotes/$(SCHEMA_REMOTE)/$$BR"; \
	if git show-ref --verify --quiet "$$REMOTE_REF"; then \
		REMOTE_TREE=$$(git rev-parse "$(SCHEMA_REMOTE)/$$BR:"); \
		echo "remote tree => $$REMOTE_TREE"; \
		if [ "$$LOCAL_TREE" = "$$REMOTE_TREE" ]; then \
			echo "No diff vs message-schema $$BR. Skip."; \
			exit 0; \
		fi; \
		# 기존 브랜치가 있으면 worktree로 증분 커밋
		WORK_DIR=$$(mktemp -d); \
		trap "git worktree remove --force '$$WORK_DIR' 2>/dev/null || true" EXIT; \
		git worktree add --detach "$$WORK_DIR" "$(SCHEMA_REMOTE)/$$BR"; \
		(cd "$$WORK_DIR" && \
			rm -rf * && \
			git read-tree "$$LOCAL_TREE" && \
			git checkout-index -af && \
			git add -A && \
			git commit -m "Update schemas from main repo @ $$(git -C .. rev-parse --short HEAD)" && \
			git push "$(SCHEMA_REMOTE)" "HEAD:refs/heads/$$BR"); \
	else \
		# 브랜치가 없으면 subtree split으로 새로 생성
		echo "No remote branch. Creating: $$BR"; \
		TMP="$$BR-tmp"; \
		git branch -D "$$TMP" 2>/dev/null || true; \
		git subtree split --prefix="$(SCHEMA_DIR)" -b "$$TMP"; \
		git push "$(SCHEMA_REMOTE)" "$$TMP:refs/heads/$$BR"; \
		git branch -D "$$TMP"; \
	fi; \
	echo "Pushed to message-schema: $$BR"
