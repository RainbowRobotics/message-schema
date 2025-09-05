ROOT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
WORKDIR := $(ROOT_DIR)
PORT ?= $(shell grep ^PORT= ${WORKDIR}/services/${SERVICE}/config.env | cut -d '=' -f2)

default:
	@echo "WORKDIR is $(WORKDIR)"

.PHONY: backend.sync
backend.sync: ## uv sync
	@cd $(WORKDIR) && uv sync --project ./backend

.PHONY: backend.lint
backend.lint: ## python ruff로 lint check 후 fix
	@cd $(WORKDIR) && \
	find . -path './.venv' -prune -o -name '*.py' -print0 \
	| xargs -0 uv run ruff check --fix --force-exclude

.PHONY: backend.dev
backend.dev: backend.lint ## Backend 개발 환경 실행
	@cd $(WORKDIR)/services/manipulate && uv run uvicorn app.main:app --reload

.PHONY: backend.build
backend.build: backend.lint
	@bash -c '\
	if [ -z "$${SERVICE}" ]; then \
		echo "🔄 모든 서비스에 대해 빌드 중..."; \
		svcs=$$(find ${WORKDIR}/services -name pyproject.toml -exec dirname {} \; | xargs -n1 basename); \
	else \
		echo "📦 서비스 '\''$${SERVICE}'\'' 빌드 중..."; \
		svcs="$${SERVICE}"; \
	fi; \
	for s in $$svcs; do \
		for arch in amd64 arm64; do \
			echo "📦 $$s ($$arch) 빌드 → 바이너리만 추출"; \
			outdir="services/$$s/.out-$$arch"; \
			echo "outdir: $$outdir"; \
			rm -rf "$$outdir" && mkdir -p "$$outdir"; \
			DOCKER_BUILDKIT=1 docker buildx build \
			  --platform "linux/$$arch" \
			  --build-arg SERVICE="$$s" \
			  -f ${WORKDIR}/Dockerfile.build \
			  --target artifacts \
			  --output "type=local,dest=$$outdir" \
			  . >/dev/null; \
			mv "$$outdir/run.bin" "${WORKDIR}/services/$$s/$$s.$$arch.bin"; \
			rm -rf "$$outdir"; \
		done; \
	done'

.PHONY: backend.ls
backend.ls:
	echo "WORKDIR is $(WORKDIR)"
	cd $(WORKDIR) && ls -al

# .PHONY: backend.build
# backend.build: ## Build backend
# 	uv run pyinstaller --onefile --clean \
# 		--distpath=./dist \
# 		--name=run.bin \
# 		./backend/services/manipulate/run.py

# .PHONY: backend.run