# Isaac Sim 자산

이 디렉터리는 현재 통합 실행에 필요한 기본 월드와 그 상대 참조 자산을 담는다.
폴더 구조를 바꾸면 USD 참조가 깨질 수 있으므로 파일을 개별적으로 다른 위치에
복사하지 않는다.

주요 진입점:

- `ing_library_env_v5-test.usd`: `world_loader.py`가 기본으로 여는 최종 통합 월드
- `Nova_Carter_ROS.usd`: Nova Carter + M0609 + RG2 결합 로봇
- `cobot3_ws/isaacpjt/M0609/`: M0609/RG2 USD와 Lula descriptor/URDF
- `book_dataset/`: 책·트레이·서가·텍스처

USD와 이미지 대부분은 Git LFS 대상이다. 새로 클론한 뒤 아래를 실행한다.

```bash
git lfs pull
./scripts/setup_check.sh
```

다른 시험 월드를 열 때만 `SIM_USD=/절대/경로.usd`를 지정한다. 기본 실행에는
홈 디렉터리나 바탕화면의 외부 자산 폴더가 필요하지 않다.
