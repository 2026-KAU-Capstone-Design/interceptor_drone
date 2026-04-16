<!--
PR 제목 규칙: <type>(<scope>): <summary>
  type   feat | fix | docs | sim | chore | refactor | test
  scope  control | perception | planning | simulation | infra | msgs | docs
  예     feat(control): add point-to-point navigation node
-->

## Summary

<!-- 무엇을, 왜 변경했는지 2~3문장 -->

## 관련 이슈

<!-- Closes #N, Relates to #M -->

## 변경 내용

<!-- 파일 단위 또는 기능 단위 bullet -->

- 
- 

## 테스트 / 검증

<!-- 어떻게 확인했는지. SITL 실행 결과, rosbag, 수치 -->

- [ ] `colcon build --packages-select <pkg>` 통과
- [ ] SITL 무풍 환경에서 동작 확인
- [ ] SITL 유풍 환경에서 동작 확인 (해당되는 경우)
- [ ] 새 문서/README 작성 또는 갱신 (해당되는 경우)

## 스크린샷 / 그래프 (선택)

<!-- 드래그앤드롭으로 이미지 첨부 -->

## 리뷰어 유의사항

<!-- 특별히 봐줬으면 하는 부분, 리뷰 시 실행해봐야 하는 명령 -->

## 체크리스트

- [ ] `feature/<이름>-<설명>` 브랜치에서 작업함
- [ ] `dev` 브랜치로 머지 요청함 (main 직접 금지)
- [ ] 커밋 메시지 prefix 규칙 준수 (`feat:`, `fix:`, `docs:` 등)
- [ ] PX4-Autopilot 본체 수정 없음 (airframe 패치 제외)
- [ ] `install/`, `build/`, `log/`, `*.pt` 등 산출물이 커밋되지 않음
