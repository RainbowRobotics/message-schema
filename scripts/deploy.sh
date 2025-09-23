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

# hotfix 여부를 선택하는 함수
function ask_hotfix() {
    local hotfix_response="no"  # 기본값을 "no"로 설정

    print_string "info" "이 배포가 hotfix 인가요? (y나 yes면 핫픽스, 아니면 Enter):" > /dev/tty

    read -r input < /dev/tty
    input=$(echo "$input" | tr '[:upper:]' '[:lower:]')  # 입력을 소문자로 변환

    if [[ "$input" == "y" || "$input" == "yes" ]]; then
        hotfix_response="yes"
    fi

    echo "$hotfix_response"
}

# Git tag 작업 수행
function git_tag_work() {
    local tag_version=$1
    local release_message=$2

    echo "tag_version: $tag_version"
    echo "release_message: $release_message"

    # 태그 생성
    if ! git tag -a "$tag_version" -m "$release_message"; then
        print_string "error" "Git tag 생성 실패"
        return 1
    fi

    # 명확히 refs/tags/ 경로로 푸시
    if ! git push origin "refs/tags/$tag_version"; then
        print_string "error" "Git tag 푸시 실패"
        return 1
    fi

    git tag -d "$tag_version"

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

    # Release 생성
    gh release create "$tag_version" \
        --title "Release $tag_version" \
        --notes "$release_message" \
        --repo "$(git config --get remote.origin.url | sed 's/.*github.com[:/]//' | sed 's/\.git$//')" || {
            print_string "error" "GitHub Release 생성 실패"
            return 1
        }
    
    print_string "success" "GitHub Release가 생성되었습니다: $tag_version"

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

# app.json 파일 버전 조회 후 새로운 버전 생성
function get_app_version() {
    local version_type=$1

    git fetch --tags > /dev/null

    version_tag=$(git tag --list 'release/[0-9]*.[0-9]*.[0-9]*' --sort=-v:refname | head -n 1)

    if [ -z "$version_tag" ]; then
        app_version="1.5.0"
    else
        app_version="${version_tag#release/}"
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
    local new_version=$1
    
    print_string "success" "=================================="
    print_string "success" "✨🔫 Slamnav2 v $new_version 배포 시작 ✨"
    print_string "success" "=================================="
}


#######################
# 메인 스크립트 실행 #
#######################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

if [ ! -f "$PROJECT_ROOT/src/comm/comm_rrs.cpp" ]; then
    echo "❌ comm_rrs.cpp 파일이 존재하지 않습니다. (경로: $PROJECT_ROOT/src/comm/comm_rrs.cpp)"
    exit 1
fi

# 주석 라인은 제외하고, io->connect("...")의 첫 번째 인자(URL)만 추출
connect_urls=$(awk '
  /^[[:space:]]*\/\// { next }  # //로 시작하는 주석 라인 스킵
  {
    if (match($0, /io->connect\("[^"]*"/, m)) {
      s = m[0]
      sub(/^.*io->connect\("/, "", s)
      sub(/"$/, "", s)
      print s
    }
  }
' "$PROJECT_ROOT/src/comm/comm_rrs.cpp" | head -n 1)

if echo "$connect_urls" | grep -q "localhost"; then
    echo "✅ rrs_socket->connect에 localhost 연결 확인."
else
    echo "❌ [comm_rrs.cpp] io->connect 주소가 '${connect_urls}' 로 되어 있습니다. ws://localhost:11337 로 수정해주세요!"
    exit 1
fi

timestamp=$(date +%Y%m%d%H%M%S)
current_branch=$(git symbolic-ref --short HEAD 2>/dev/null || git name-rev --name-only HEAD)

hotfix_check_branches=("main" "develop")

if [[ "$current_branch" = "main" ]]; then
    # GitHub CLI가 설치되어 있는지 확인
    if ! command -v gh >/dev/null 2>&1; then
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

# 현재 버전 생성
new_version="$timestamp"

if [[ " ${hotfix_check_branches[*]} " =~ " $current_branch " ]]; then
     # 메인 스크립트 실행 부분에 hotfix 여부 확인 추가
    is_hotfix=$(ask_hotfix)
    echo "Hotfix 여부: $is_hotfix"
fi

if [[ "$current_branch" = "main" ]]; then
    version_type=$(ask_version_type)
    echo "버전 타입: $version_type"

    main_release_message=$(get_release_message)

    new_version=$(get_app_version $version_type)
    new_version=$(echo "$new_version" | tr -d '\n' | tr -d '\r')
fi

# 태그 버전 생성
if [ "$is_hotfix" = "yes" ]; then
    tag_version="deploy_hotfix/${current_branch}/${new_version}"
else
    tag_version="deploy/${current_branch}/${new_version}"
fi

# 릴리즈 메시지 생성
release_message="[App: SLAMNAV2, Version: ${new_version}] release 배포"


# Git 작업 실행
git_tag_work "$tag_version" "$release_message" || last_git_work_status="bad"

if [[ "$last_git_work_status" = "bad" ]]; then
    git tag -d $tag_version
    git push origin --delete $tag_version
fi

# 최종 상태에 따라 종료
if [ "$last_git_work_status" = "normal" ]; then
    if [[ "$current_branch" = "main" ]]; then
        git_tag_work "release/${new_version}" "$main_release_message"
        create_github_release "$new_version" "$main_release_message"
    fi

    if [ "$last_git_work_status" = "bad" ]; then
        git tag -d $tag_version
        git push origin --delete $tag_version
        git tag -d $release_tag
        git push origin --delete $release_tag
        print_string "error" "배포 실패"
        exit 1
    else
        print_completion_message "$new_version"
        exit 0
    fi
else
    exit 1
fi
