#!/bin/bash

###################
# 유틸리티 함수들 #
###################

# 컬러 출력 함수
function print_string(){
  local RED='\033[0;31m'
  local GREEN='\033[0;32m'
  local YELLOW='\033[1;33m'
  local NC='\033[0m'

  case "$1" in
    "error") echo -e "${RED}${2}${NC}" ;;
    "success") echo -e "${GREEN}${2}${NC}" ;;
    "warning") echo -e "${YELLOW}${2}${NC}" ;;
    "info") echo -e "${NC}${2}${NC}" ;;
  esac
}

# 서비스 이름 입력 받는 함수
function get_service_name() {
    local app_names=()
    local app_dirs=()
    local host_deploy="no"

    # print_string 출력을 /dev/tty로 강제 지정
    print_string "info" "배포할 서비스 이름을 입력해주세요 (여러 서비스는 쉼표로 구분, 전체는 all):" > /dev/tty

    # 한 줄만 읽도록 수정
    read -e input < /dev/tty

    if [ -z "${input}" ]; then
        print_string "error" "서비스 이름을 입력해주세요." > /dev/tty
        get_service_name
    fi

    # 입력값이 'all'인 경우
    if [ "${input}" = "all" ]; then
        # apps/web 디렉토리의 모든 하위 디렉토리를 찾아서 처리
        for dir in backend/services/*/; do
            if [ -d "${dir}" ]; then
                # 디렉토리 경로에서 앱 이름만 추출
                local app_name=$(basename "${dir}")
                # 중복 체크 후 추가
                if [[ ! " ${app_names[@]} " =~ " ${app_name} " ]]; then
                    app_names+=("$app_name")
                    app_dirs+=("backend/services/$app_name")
                fi
            fi
        done
        # host 배포 포함
        app_names+=("host")
        app_dirs+=("backend/host")
        host_deploy="yes"
    else
        # 기존 로직: 쉼표로 구분된 입력을 배열로 변환
        IFS=',' read -ra input_array <<< "${input}"

        for app_name in "${input_array[@]}"; do
            # 앱 이름에서 공백 제거
            app_name=$(echo "${app_name}" | xargs)
            if [ "$app_name" = "host" ]; then
                host_deploy="yes"
                app_names+=("host")
                app_dirs+=("backend/host")
            elif [ -d "backend/services/${app_name}" ] && [[ ! " ${app_names[@]} " =~ " ${app_name} " ]]; then
                app_names+=("${app_name}")
                app_dirs+=("backend/services/${app_name}")
            else
                print_string "warning" "'backend/services/${app_name}' 디렉토리가 존재하지 않습니다." > /dev/tty
                get_service_name
            fi
        done
    fi

    # 배열을 구분자로 구분하여 문자열로 변환하여 반환
    local names_str=$(IFS=','; echo "${app_names[*]}")
    local dirs_str=$(IFS=','; echo "${app_dirs[*]}")

    echo "${names_str} ${dirs_str} ${host_deploy}"
}

# hotfix 여부를 선택하는 함수
function ask_hotfix() {
    local hotfix_response="no"  # 기본값을 "no"로 설정

    print_string "info" "이 배포가 hotfix 인가요? (y나 yes면 핫픽스, 아니면 Enter):" > /dev/tty

    read -r input < /dev/tty
    input=$(echo "${input}" | tr '[:upper:]' '[:lower:]')  # 입력을 소문자로 변환

    if [[ "${input}" == "y" || "${input}" == "yes" ]]; then
        hotfix_response="yes"
    fi

    echo "${hotfix_response}"
}

# .bin 추적 해제 및 ignore 복구 커밋
function git_cleanup_bin_track() {
    local -n app_names=$1

    local bin_files=()

    for app_name in "${app_names[@]}"; do
        bin_files+=($(find backend/services/${app_name} -type f -name "*.bin"))
        bin_files+=($(find backend/host -type f -name "*.bin"))
    done

    if [ ${#bin_files[@]} -gt 0 ]; then
        print_string "info" "🧹 .bin 추적 해제 및 ignore 복구 커밋..."
        for bf in "${bin_files[@]}"; do
            git rm --cached -q "$bf" || true
        done
        git commit -m "chore: untrack binaries after deploy" || true
        git push origin || true
    fi
}

# Git tag 작업 수행
function git_tag_work() {
    local new_version=$1
    local tag_version=$2
    local app_dir=$3
    local app_name=$4
    local release_message=$5

    if git rev-parse -q --verify "refs/tags/${tag_version}" >/dev/null 2>&1; then
        print_string "warning" "'${tag_version}' 로컬 태그가 이미 있어 삭제합니다"
        git tag -d "${tag_version}" || true
    fi

    if git ls-remote --tags origin | awk '{print $2}' | grep -Fxq "refs/tags/${tag_version}"; then
        print_string "warning" "'${tag_version}' 원격 태그가 이미 있어 삭제합니다"
        git push origin ":refs/tags/${tag_version}" || true
    fi

    if ! git diff --cached --name-only | grep -q "api-gateway"; then
        git add -f "api-gateway" || { print_string "error" "Git add api-gateway 실패"; return 1; }
    fi

    # bin 강제 add
    if [ -f "${app_dir}/${app_name}.arm64.bin" ] && [ $app_dir != "." ]; then
        git add -f "${app_dir}/${app_name}.arm64.bin" || { print_string "error" "Git add ${app_name}.arm64.bin 실패"; return 1; }
    fi

    if [ -f "${app_dir}/${app_name}.amd64.bin" ] && [ $app_dir != "." ]; then
        git add -f "${app_dir}/${app_name}.amd64.bin" || { print_string "error" "Git add ${app_name}.amd64.bin 실패"; return 1; }
    fi

    if [ -f "${app_dir}/${app_name}.bin" ] && [ $app_dir != "." ]; then
        echo "git add -f ${app_dir}/${app_name}.bin"
        git add -f "${app_dir}/${app_name}.bin" || { print_string "error" "Git add ${app_name}.bin 실패"; return 1; }
    fi

    # 태그용 임시 커밋
    git commit --allow-empty -m "chore: Deploy ${app_name} ${new_version}" || { print_string "error" "Git commit 실패"; return 1; }


    # 태그 생성 + push
    git tag -a "$tag_version" -m "$release_message" || { print_string "error" "Git tag 실패"; return 1; }
    git push origin "$tag_version" || { print_string "error" "Git tag push 실패"; return 1; }
    echo "git push origin $tag_version"

    # 태그 로컬 제거
    git tag -d "$tag_version" || { print_string "error" "Git tag 삭제 실패"; return 1; }

    return 0
}

function git_tag_work_total() {
    local new_version=$1
    local tag_version=$2
    local release_message=$3

    if git rev-parse -q --verify "refs/tags/${tag_version}" >/dev/null 2>&1; then
        print_string "warning" "'${tag_version}' 로컬 태그가 이미 있어 삭제합니다"
        git tag -d "${tag_version}" || true
    fi

    # 태그 생성 + push
    git tag -a "$tag_version" -m "$release_message" || { print_string "error" "Git tag 실패"; return 1; }
    git push origin "$tag_version" || { print_string "error" "Git tag push 실패"; return 1; }

    # 태그 로컬 제거
    git tag -d "$tag_version" || { print_string "error" "Git tag 삭제 실패"; return 1; }

    return 0
}



# 빌드 프로세스
function build_project() {
    local app_names=$1

    echo "app_names: ${app_names}"

    print_string "info" "프로젝트 빌드 중..."

    make backend.build SERVICE="${app_names}" || {
        print_string "error" "빌드 실패"
        return 1
    }
    print_string "success" "패키지 설치 및 빌드 완료"
    return 0
}

# 버전 타입 선택 함수

function ask_version_type() {
    local version_type="patch"

    print_string "info" "배포할 버전 타입을 선택해주세요 (major/minor/patch, 기본값: patch):" > /dev/tty

    read -r input < /dev/tty
    input=$(echo "$input" | tr '[:upper:]' '[:lower:]')

    if [[ "$input" == "major" || "$input" == "minor" || "$input" == "patch" ]]; then
        version_type="$input"
    fi

    echo "$version_type"
}

# GitHub Release 생성 함수
function create_github_release() {
    local tag_version=$1
    local release_message=$2

    local app_name=$(echo "$tag_version" | cut -d'/' -f2)

    # Release 생성
    gh release create "$tag_version" \
        --title "Release $tag_version" \
        --notes "$release_message" \
        --repo "$(git config --get remote.origin.url | sed 's/.*github.com[:/]//' | sed 's/\.git$//')" || {
            print_string "error" "GitHub Release 생성 실패"
            return 1
        }

    print_string "success" "[${app_name}] ✅ GitHub Release가 생성되었습니다: $tag_version"

    return 0
}

# 작업 내역 입력 받는 함수
function get_release_message() {
    local release_message=""

    # print_string 출력을 /dev/tty로 강제 지정
    print_string "info" "작업 내역을 입력해주세요 (필수, 여러 줄 입력. 입력 완료 시 Enter를 치고 Control+D를 입력하세요):" > /dev/tty

    # read -e를 사용하여 편집 가능한 입력 받기
    while IFS= read -e line; do
        release_message+="$line"$'\n'
    done < /dev/tty

    release_message_trimmed=$(echo "$release_message" | tr -d '[:space:]')
    if [ -z "$release_message_trimmed" ]; then
        print_string "error" "작업 내역이 비어있습니다. 다시 입력해주세요." > /dev/tty
        release_message=$(get_release_message)
    fi

    echo "$release_message"
}

# Release 버전 조회 후 새로운 버전 생성
function get_app_version() {
    local version_type=$1
    local app_name=$2

    git fetch --tags > /dev/null

    version_tag="$(
        git tag --list \
            "release/${app_name}/*/[0-9]*.[0-9]*.[0-9]*" \
            "release/${app_name}/[0-9]*.[0-9]*.[0-9]*" \
            --sort=-v:refname | head -n1
        )"

    if [ -z "$version_tag" ]; then
        app_version="0.0.1"
    else
        app_version="${version_tag##*/}"
    fi

    IFS='.' read -r major minor patch <<< "$app_version"

    case "$version_type" in
        "major")
            major=$((major + 1))
            minor=0
            patch=0
            ;;
        "minor")
            minor=$((minor + 1))
            patch=0
            ;;
        *)
            patch=$((patch + 1))
            ;;
    esac

    new_version="${major}.${minor}.${patch}"

    echo "$new_version"
}

# 완료 메시지 출력
function print_completion_message() {
    local -n new_versions=$1

    print_string "success" "=================================="

    for version in "${new_versions[@]}"; do
        print_string "success" "✨🎉 $version 배포 완료 🎉✨"
    done

    print_string "success" "=================================="
}

#######################
# 메인 스크립트 실행 #
#######################

if [[ "$current_branch" = "main" ]]; then
    # GitHub CLI가 설치되어 있는지 확인
    if ! command -v gh &> /dev/null; then
        print_string "error" "GitHub CLI가 설치되어 있지 않습니다."
        exit 1
    fi

    # GitHub CLI 로그인 상태 확인
    if ! gh auth status &> /dev/null; then
        print_string "error" "GitHub CLI에 로그인되어 있지 않습니다. 'gh auth login'으로 로그인해주세요."
        exit 1
    fi
fi

# 현재 브랜치 저장
last_git_work_status="normal"

timestamp=$(date +%Y%m%d%H%M%S)
current_branch=$(git rev-parse --abbrev-ref HEAD)

# 배포할 서비스 이름 입력 받기
# 메인 스크립트에서 배열로 변환하여 사용
read names_str dirs_str host_deploy <<< $(get_service_name)

# 공백 제거
names_str=$(echo "${names_str}" | xargs)
dirs_str=$(echo "${dirs_str}" | xargs)
host_deploy=$(echo "${host_deploy}" | xargs)

if [[ "$current_branch" = "main" ]]; then
    # 메인 스크립트 실행 부분에 hotfix 여부 확인 추가
    is_hotfix=$(ask_hotfix)
    echo "Hotfix 여부: $is_hotfix"

    version_type=$(ask_version_type)
    echo "버전 타입: $version_type"

    release_message=$(get_release_message)
fi

# 문자열을 다시 배열로 변환
IFS=',' read -ra app_names <<< "$names_str"
IFS=',' read -ra app_dirs <<< "$dirs_str"

echo "app_names: ${app_names[@]}"
echo "app_dirs: ${app_dirs[@]}"
echo "host_deploy: ${host_deploy}"

# 빌드 실행
# build_project "$names_str" || exit 1

new_versions=()

# 각 앱에 대해 반복 처리
for i in "${!app_names[@]}"; do
    app_name="${app_names[$i]}"
    app_dir="${app_dirs[$i]}"

    echo "app_name: ${app_name}"

    if [[ "$last_git_work_status" = "bad" ]]; then
        break
    fi

    # 현재 버전 생성
    new_version="${app_name}-${timestamp}"

    if [[ "$current_branch" = "main" ]]; then
        new_version=$(get_app_version $version_type $app_name)
    fi

    echo "new_version: ${new_version}"

    # 태그 버전 생성
    if [ "$is_hotfix" = "yes" ]; then
        tag_version="deploy_hotfix/${app_name}/${current_branch}/${new_version}"
    else
        tag_version="deploy/${app_name}/${current_branch}/${new_version}"
    fi

    echo "tag_version: ${tag_version}"

    # 배포 메시지 생성
    deploy_message="deploy: [App: ${app_name}, Version: ${new_version}] release 배포"

    if [[ "$current_branch" = "main" ]] && [[ "$last_git_work_status" = "normal" ]]; then
        release_tag_version="release/${app_name}/${new_version}"
        git_tag_work "$new_version" "$release_tag_version" "$app_dir" "$app_name" "$release_message" || last_git_work_status="bad"
        create_github_release "$release_tag_version" "$release_message" || last_git_work_status="bad"
    fi

    if [[ "$last_git_work_status" = "normal" ]]; then
        echo "git_tag_work!!!"
        # Git 작업 실행
        git_tag_work "$new_version" "$tag_version" "$app_dir" "$app_name" "$deploy_message" || last_git_work_status="bad"
    fi


    if [[ "$last_git_work_status" = "bad" ]]; then
        echo "git work status BAD!!!"
        git tag -d $tag_version
        git push origin --delete $tag_version
    else
        new_versions+=("${app_name}: v ${new_version}")
    fi
done

echo "last_git_work_status!@: ${last_git_work_status}"

# .bin 추적 해제 및 ignore 복구 커밋
git_cleanup_bin_track "${app_names}"

echo "git_cleanup_bin_track!!!"

if [[ "$last_git_work_status" = "normal" ]]; then
    sleep 10

    if [[ "$current_branch" = "main" ]]; then
        new_total_version=$(get_app_version $version_type "total")
        release_tag_version="release/total/${current_branch}/${new_total_version}"
        git_tag_work_total "$new_total_version" "$release_tag_version" "$release_message" || last_git_work_status="bad"
        create_github_release "$release_tag_version" "$release_message" || last_git_work_status="bad"

        if [[ "$last_git_work_status" = "normal" ]]; then
            new_versions+=("v $new_total_version")
        fi
    else
        release_tag_version="release/dev-total/${current_branch}/${timestamp}"
        echo "release_tag_version: ${release_tag_version}"
        git_tag_work_total "$new_total_version" "$release_tag_version" "$release_message" || last_git_work_status="bad"
        echo "git_tag_work_total!!!"
    fi

    if [[ "$last_git_work_status" = "bad" ]]; then
        echo "last_git_work_status BAD!!!"
        git tag -d $release_tag_version
        git push origin --delete $release_tag_version
    fi
fi

# 완료 메시지 출력
if [ "$last_git_work_status" = "normal" ]; then
    echo "last_git_work_status NORMAL!!!"
    echo "new_versions: ${new_versions[@]}"
    print_completion_message "$new_versions"
    exit 0
else
    exit 1
fi
