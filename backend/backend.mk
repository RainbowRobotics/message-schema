PY ?= python
SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

WORKDIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROOT_DIR := $(cd "${WORKDIR}/.." && pwd)
CONF_PORT ?= $(shell grep ^PORT= ${WORKDIR}/services/${SERVICE}/config.env | cut -d '=' -f2)

.PHONY: backend.sync
backend.sync: ## uv sync
	@cd ${WORKDIR} && uv sync --all-packages --frozen

.PHONY: backend.add
backend.add: ## uv add - SERVICE or PACKAGE 로 설치할 위치 지정 필수, DEPS 는 설치할 의존성 지정 필수 (ex. backend.add SERVICE=manipulate DEPS=fastapi)
	@bash -c '\
	if [ -n "$(PACKAGE)" ]; then \
		echo "📦 패키지 '$(PACKAGE)'에 러닝타임 의존성 추가 중: $(DEPS)"; \
		cd ${WORKDIR}/packages/${PACKAGE} && uv add ${IS_DEV:--dev} ${DEPS}; \
	elif [ -n "$(SERVICE)" ]; then \
		echo "📦 서비스 '$(SERVICE)'에 러닝타임 의존성 추가 중: $(DEPS)"; \
		cd ${WORKDIR}/services/${SERVICE} && uv add ${IS_DEV:--dev} ${DEPS}; \
	else \
		echo "📦 루트에 개발용 의존성 추가 중: ${DEPS}"; \
		cd ${WORKDIR} && uv add --dev ${DEPS}; \
	fi'

.PHONY: backend.lint
backend.lint: ## python ruff로 lint check 후 fix
	@cd ${WORKDIR} && uv run ruff check . --fix --force-exclude

.PHONY: backend.local-dev
backend.local-dev: backend.lint backend.flatc ## docker를 쓰지 않고 개발 환경 실행
	@cd ${WORKDIR}/services/${SERVICE} && uv run uvicorn app.main:app --host 0.0.0.0 --port ${CONF_PORT} --reload --reload-dir ${WORKDIR} --reload-include '**/*.py'

.PHONY: backend.dev
backend.dev: backend.lint ## docker를 쓰고 개발 환경 실행
	@bash ${ROOT_DIR}api-gateway/generate-nginx-conf.sh --dev
	@bash ${ROOT_DIR}scripts/backend/generate-compose.sh
	@bash -euo pipefail -c '\
		set -Eeuo pipefail; \
		compose="docker compose -f ${WORKDIR}/docker-compose.dev.yml"; \
		$${compose} pull --ignore-buildable || true; \
		if $${compose} build; then \
		  exec $${compose} up; \
		else \
		  exec $${compose} up --no-build; \
		fi'

.PHONY: backend.build
backend.build: backend.lint ## 모든 Backend 서비스 또는 지정된 Backend 서비스 빌드
	@bash scripts/backend/build.sh


.PHONY: backend.preview
backend.preview: ## Backend 운영 환경 실행
	# @docker build -t rrs-nginx:latest api-gateway/
	@bash ${ROOT_DIR}api-gateway/generate-nginx-conf.sh
	@bash ${ROOT_DIR}scripts/backend/generate-compose.sh
	@docker compose -f ${WORKDIR}/docker-compose.preview.yml up --build

.PHONY: backend.flatc
backend.flatc: ## FlatBuffers 코드 생성
	echo "WORKDIR: $(WORKDIR)"
	echo "ROOT_DIR: $(ROOT_DIR)"
	@if command -v flatc >/dev/null 2>&1; then \
		find "$(WORKDIR)/packages/rb_flat_buffers/src/rb_flat_buffers" -mindepth 1 -exec rm -rf {} +; \
		find "${WORKDIR}/../schemas" -name "*.fbs" -exec flatc --python --gen-object-api --gen-all --python-typing --python-gen-numpy -o "${WORKDIR}/packages/rb_flat_buffers/src/rb_flat_buffers" {} \; ; \
		$(PY) "${WORKDIR}/packages/rb_flat_buffers/scripts/patch_imports.py" \
		  "${WORKDIR}/packages/rb_flat_buffers/src/rb_flat_buffers"; \
		find "${WORKDIR}/packages/rb_flat_buffers/src/rb_flat_buffers" -type d -exec sh -c 'for d in "$$@"; do : > "$$d/__init__.py"; done' _ {} +; \
	else \
		echo "‼️ flatc를 설치해주세요!"; \
		exit 1; \
	fi

.PHONY: backend.deploy
backend.deploy:  ## 모든 Backend 서비스 또는 지정된 Backend 서비스 배포
	@bash ${ROOT_DIR}api-gateway/generate-nginx-conf.sh
	@bash ${ROOT_DIR}scripts/backend/deploy.sh

backend.mypy: ## mypy로 type check
	@echo "🔍 Type checking backend services..."
	@rm -rf ${WORKDIR}/.mypy_cache
	@find ${WORKDIR}/services -maxdepth 1 -type d ! -path ${WORKDIR}/services -exec basename {} \; | while read service; do \
		echo "📦 Checking $$service..."; \
		cd ${WORKDIR} && uv run mypy services/$$service \
			--explicit-package-bases \
			--config-file ${WORKDIR}/pyproject.toml || true; \
	done
	@echo "🔍 Type checking backend packages..."
	@find ${WORKDIR}/packages -maxdepth 1 -type d ! -path ${WORKDIR}/packages -exec basename {} \; | while read package; do \
		echo "📦 Checking $$package..."; \
		cd ${WORKDIR} && uv run mypy -p $$package \
			--explicit-package-bases \
			--config-file ${WORKDIR}/pyproject.toml || true; \
	done
	@echo "✅ Type check completed"
