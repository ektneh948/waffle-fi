#!/usr/bin/env bash
set -euo pipefail

echo "== GitHub SSH 빠른세팅 (2601_4th_proj_dahyeon) =="

# ORG는 고정
ORG="intel-edge-ai-sw-8"

# REPO와 EMAIL은 반드시 사용자가 입력
REPO="2601_4th_proj_dahyeon"
# while [[ -z "${REPO}" ]]; do
#   read -rp "본인 전용 REPO 이름 (예: dooreu) [필수]: " REPO
# done

EMAIL=""
while [[ -z "${EMAIL}" ]]; do
  read -rp "본인 GitHub 계정 이메일 [필수]: " EMAIL
done

echo
echo "Org : $ORG"
echo "Repo: $REPO"
echo "Email: $EMAIL"
echo

# [1] 필수 패키지 확인/설치 (git, ssh)
need_install=false
command -v git >/dev/null 2>&1 || need_install=true
command -v ssh >/dev/null 2>&1 || need_install=true

if $need_install; then
  echo "[1/10] git/ssh 설치 중..."
  if command -v apt >/dev/null 2>&1; then
    sudo apt update && sudo apt install -y git openssh-client
  elif command -v dnf >/dev/null 2>&1; then
    sudo dnf install -y git openssh-clients
  elif command -v pacman >/dev/null 2>&1; then
    sudo pacman -Sy --noconfirm git openssh
  elif command -v zypper >/dev/null 2>&1; then
    sudo zypper install -y git openssh
  elif command -v brew >/dev/null 2>&1; then
    brew install git openssh
  else
    echo "❌ 자동 설치를 지원하지 않는 배포판입니다. git/openssh를 수동 설치해주세요."
    exit 1
  fi
else
  echo "[1/10] git/ssh 이미 설치됨"
fi

# [2] Git 사용자 정보 설정(처음 사용자용)
if ! git config --global user.email >/dev/null 2>&1; then
  git config --global user.email "$EMAIL"
fi
if ! git config --global user.name >/dev/null 2>&1; then
  DEFAULT_NAME="${EMAIL%@*}"
  read -rp "Git 사용자 이름(커밋에 표시) [기본: $DEFAULT_NAME]: " GNAME || true
  git config --global user.name "${GNAME:-$DEFAULT_NAME}"
fi

# [3] SSH 키 생성(없을 경우만)
KEY="$HOME/.ssh/id_ed25519"
if [[ -f "$KEY" ]]; then
  echo "[2/10] SSH 키 이미 존재: $KEY"
else
  echo "[2/10] SSH 키 생성: $KEY"
  mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"
  # 보안 강화를 원하면 -N "" 제거 후 프롬프트에서 패스프레이즈 입력
  ssh-keygen -t ed25519 -C "$EMAIL" -f "$KEY" -N ""
fi

# [4] ssh-agent 시작 및 키 등록 + ~/.ssh/config 설정
echo "[3/10] ssh-agent 시작 및 키 등록"
eval "$(ssh-agent -s)" >/dev/null
ssh-add "$KEY" >/dev/null || true

CFG="$HOME/.ssh/config"
if ! grep -q "^Host github.com" "$CFG" 2>/dev/null; then
  echo "[4/10] ~/.ssh/config에 github.com 항목 추가"
  {
    echo "Host github.com"
    echo "  HostName github.com"
    echo "  User git"
    echo "  IdentityFile $KEY"
    echo "  AddKeysToAgent yes"
  } >> "$CFG"
  chmod 600 "$CFG"
else
  echo "[4/10] ~/.ssh/config에 github.com 항목 이미 존재 (건너뜀)"
fi

# [5] 공개키 출력 → GitHub에 등록 안내
echo "[5/10] 아래 공개키를 GitHub에 등록하세요 (Settings → SSH and GPG keys → New SSH key)"
echo "----- COPY BELOW -----"
cat "${KEY}.pub"
echo "----- COPY ABOVE -----"
echo
read -rp "GitHub에 SSH 키를 추가(SSO 강제 Org면 Authorize까지)한 뒤 Enter를 누르세요..." _

# [6] SSH 연결 테스트 (처음이면 yes 한 번)
echo "[6/10] SSH 연결 테스트"
ssh -T git@github.com || true

# [7] 레포 접근 확인(SSH만 사용)
echo "[7/10] 레포 접근 확인: ${ORG}/${REPO}"
if ! GIT_SSH_COMMAND="ssh -o BatchMode=yes" git ls-remote "git@github.com:${ORG}/${REPO}.git" &>/dev/null; then
  echo "❌ 오류: ${ORG}/${REPO} 에 SSH로 접근할 수 없습니다."
  echo "   - 공개키 등록/승인(SSO) 확인"
  echo "   - 레포 권한(읽기/쓰기) 확인"
  echo "   - SSO 강제 조직이면: 'ssh -T git@github.com' 출력 링크로 Authorize 수행"
  exit 1
fi
echo "✅ ${ORG}/${REPO} SSH 접근 확인 완료"

# [8] Clone 옵션 선택
# echo
# echo "[8/10] Clone 방법을 선택하세요:"
# echo "  1) cmdlogs 폴더만 sparse-checkout"
# echo "  2) 전체 repo clone"
# CHOICE=""
# while [[ "$CHOICE" != "1" && "$CHOICE" != "2" ]]; do
#   read -rp "선택 [1/2]: " CHOICE
# done
CHOICE="2"

# [9] Clone 실행
TARGET_ROOT="$HOME/$ORG"
TARGET_DIR="$TARGET_ROOT/$REPO"
mkdir -p "$TARGET_ROOT"

if [[ -d "$TARGET_DIR/.git" ]]; then
  echo "[9/10] 이미 존재: $TARGET_DIR (원격 설정만 확인)"
  (cd "$TARGET_DIR" && git remote set-url origin "git@github.com:${ORG}/${REPO}.git" && git remote -v)
else
  if [[ "$CHOICE" == "2" ]]; then
    echo "[9/10] 전체 repo clone → $TARGET_DIR"
    git clone "git@github.com:${ORG}/${REPO}.git" "$TARGET_DIR"
  else
    echo "[9/10] sparse-checkout (cmdlogs 폴더만) → $TARGET_DIR"
    git clone --no-checkout "git@github.com:${ORG}/${REPO}.git" "$TARGET_DIR"
    cd "$TARGET_DIR"
    git sparse-checkout init --cone
    git sparse-checkout set cmdlogs
    git checkout
  fi
fi

# [10] 완료 안내
echo
echo "[10/10] 완료 ✅  경로: $TARGET_DIR"
if [[ "$CHOICE" == "1" ]]; then
  echo "체크아웃된 폴더:"
  echo "  $TARGET_DIR/cmdlogs"
else
  echo "전체 repo가 clone 되었습니다."
fi

echo
echo "다음 예시:"
echo "  cd \"$TARGET_DIR/cmdlogs\""
echo "  echo hi > test.txt"
echo "  git pull origin"
echo "  git add ."
echo "  git commit -m \"test push\""
echo "  git push origin main"
