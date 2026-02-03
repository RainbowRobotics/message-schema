SCHEMA_DIR := rb_ipc/comm_fbs
SCHEMA_REMOTE := message-schema
SCHEMA_REMOTE_URL := https://github.com/RainbowRobotics/message-schema.git

.PHONY: schema-subtree-init
schema-subtree-init: ## schema subtree 최초 1회 초기화
	@[ ! -d "$(SCHEMA_DIR)" ] || (echo "❌ $(SCHEMA_DIR) 이라는 디렉토리가 이미 존재합니다. 지워주시고 커밋/푸시 후 진행해주세요!"; exit 1)
	@git remote get-url $(SCHEMA_REMOTE) >/dev/null 2>&1 \
	  || git remote add $(SCHEMA_REMOTE) $(SCHEMA_REMOTE_URL)
	@git subtree add --prefix=$(SCHEMA_DIR) $(SCHEMA_REMOTE) main --squash

.PHONY: schema-remote-add
schema-remote-add: ## schema remote add
	@if ! git remote get-url "$(SCHEMA_REMOTE)" >/dev/null 2>&1; then \
		echo "remote '$(SCHEMA_REMOTE)' 를 찾을 수 없습니다. remote를 추가합니다."; \
		git remote add "$(SCHEMA_REMOTE)" "$(SCHEMA_REMOTE_URL)"; \
	fi
	@git fetch "$(SCHEMA_REMOTE)" --prune

.PHONY: schema-update
schema-update: schema-remote-add ## schemas 변경사항을 현재 레포와 서브레포로 push 그리고 자동 PR 진행
	@bash "$(SCHEMA_DIR)/schema-update.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)

.PHONY: schema-sync
schema-sync: schema-remote-add ## message-schema의 main 브랜치 내용을 현재 SCHEMA_DIR에 그대로 가져오기 (변경 사항이 있다면 진짜 덮어씌울건지 물어봄)
	@bash "$(SCHEMA_DIR)/schema-sync.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE)


.PHONY: schema-sync-force
schema-sync-force: schema-remote-add ## message-schema의 main 브랜치 내용을 현재 SCHEMA_DIR에 그대로 가져오기 (변경 사항이 있다면 강제 덮어씌우기)
	@bash "$(SCHEMA_DIR)/schema-sync.sh" --dir $(SCHEMA_DIR) --remote $(SCHEMA_REMOTE) -y
