# Simple Room 투명 물체 센서 실험 — Isaac Sim 6.0.1

작성일: 2026-09-14, 갱신: 2026-09-15. 스크립트와 로컬 6.0.1 API 대조, 기하 계산 테스트를 완료했다.
GPU에서 에셋 로딩·렌더링·DIP/DSD 출력까지 실행 검증했다(검증 기록 참고).
**다만 실측 센서와의 대조 보정은 하지 않았다.** 이 문서의 수치들은 실험 시작값이며,
실측으로 검증된 센서 성능값이 아니다.

## 구성과 재질의 의미

- 환경: `get_assets_root_path()` + `/Isaac/Environments/Simple_Room/simple_room.usd`.
  이 경로는 로컬 6.0.1의 공식 MoveIt 예제에서도 확인했다.
- 원본 환경은 USD reference로 읽고 새 실험 장면만 출력 폴더에 저장한다.
- 표적은 컵이다. `cup_asset`(기본 `/Isaac/Props/Mugs/SM_Mug_A2.usd`)을 슬롯마다 같은 것으로 참조한다.
  에셋은 metersPerUnit=1.0, Z-up, 바닥이 로컬 z=0이며 몸통 반지름 약 46 mm, 높이 약 91 mm이다.
  네 조건의 **기하와 Collider는 완전히 동일하고 재질만 다르다.** 에셋이 메시에 직접 바인딩한 재질을
  덮어써야 하므로 슬롯 Xform에 `strongerThanDescendants`로 바인딩한다.
- `opaque`: UsdPreviewSurface, opacity=1, roughness=0.65.
- `omniglass`: OmniGlass, IOR=1.5, frosting_roughness=0.01, thin_walled=False. 빈 유리컵이다.
- `transparent`: UsdPreviewSurface, opacity=`transparent_opacity`. **단순 알파 투명 대조군이며 실제 아크릴 물성 모델이 아니다.**
  렌더러의 opacity 처리 자체도 실험 대상이다. 불투명/투명은 같은 셰이더의 opacity만 다르게 한다.
- `omniglass_water`: 컵 자체는 `omniglass`와 같고, 안에 IOR 1.333의 물 실린더를 넣는다.
  **이 실린더는 컵 메시의 실제 내부를 측정한 값이 아니라 `config.json`의 근사값이다.**
  컵 벽을 뚫거나 수면 높이가 틀리지 않는지 첫 GUI 실행에서 확인한다.
- 네 컵을 Y 방향으로 나란히 배치하고 배경 체크무늬 판을 고정한다.
  기본 방의 기존 테이블을 측정 기준으로 삼지 않고 별도 작은 실험대를 추가한다.
- `bench_origin_m`은 **실험대 윗면 중심**(기본 1.2 m)이고 컵은 그 위에 선다.
  카메라는 컵 바닥이 아니라 몸통 중심을 겨냥한다. 기존 방 가구/벽과의 겹침은 첫 GUI 실행에서 확인한다.
- 컵은 판보다 훨씬 작다. 0.8 m에서 폭이 약 64 px(판은 약 208 px)이므로
  `valid_fraction_full_image`는 대부분 배경 판을 재는 값이다. 재질 비교는 컵 영역만 잘라서 봐야 한다.
- `slot_spacing_m`이 0.25 m라 이웃 컵도 화면에 같이 들어온다. 표적만 보려면 간격을 넓힌다.
- `material_order`를 순환 변경해서 동일 재질이 다른 공간에서도 같은 결과를 내는지 확인한다.
  물체를 서로 다른 자리에 두면 방의 조명과 가림이 재질 효과와 섞일 수 있다.

## 센서 모듈

`sensors.py`의 `DepthSensorModule`이 아래 출력을 관리한다.

| mode | API | 출력의 의미 |
|---|---|---|
| dip | CameraSensor + distance_to_image_plane | 카메라 광축 Z 방향의 깊이 |
| dsd | SingleViewDepthCameraSensor + depth_sensor_distance | 단일 뷰 기반 스테레오 깊이 후처리 결과 |
| both | 동일 자세·광학 파라미터의 별도 카메라/RenderProduct 두 개 | 한 업데이트 뒤 두 출력을 수집 |
| lidar | `Lidar` + `LidarSensor` + `generic-model-output` | RTX 레이트레이싱 기반 희소 반환 |

`LidarModule`은 카메라와 같은 자세에 RTX Lidar를 놓고 같은 업데이트에서 읽는다.
**깊이 annotator와 달리 레이트레이싱 결과이고 반환마다 non-visual material id가 붙는다.**
로컬 측정에서 `matId`는 11=plastic, 19=clear_glass로 네 컵을 재질별로 구분했다.
출력은 네이티브 구면 좌표(x=azimuth deg, y=elevation deg, z=range m, 센서 프레임)이며,
`points_m`은 이를 월드 XYZ로 변환한 값이다. 이미지로 재구성하지 않는다.
Lidar는 카메라보다 `lidar.mount_height_m`(기본 0.25 m)만큼 높이 달고 같은 겨냥점을 향해 내려본다.
**카메라 높이에 그대로 두면 센서가 실험대 상판 45 mm 위에 놓여 아래쪽 링이 상판을 스치듯 긁는다.**
그러면 상판 위로 거대한 동심원 호가 생기고 상판 끝에서 잘려 반원판처럼 보인다.
기본 Lidar는 360° × 25° 회전형이고 `omni:sensor:Core:farRangeM`의 기본값이 **200 m**라서
그대로 두면 방 전체를 스캔한다. 그래서 `lidar.far_range_m`(기본 2.0 m)으로 사거리를 잘라
스캔이 실험대에 남도록 한다. 이 설정 하나로 반환이 460k → 128k로 줄고
방이 차지하는 비율이 80.5 % → 34.5 %가 된다.
`far_range_m`은 가장 먼 촬영 거리보다 커야 하며 run.py가 검사한다.
평면 위의 동심원 링 자체는 회전형 Lidar의 정상 구조다.

`module.select_mode("dip")` / `module.select_mode("dsd")` 후 `module.read()`로 전환한다.
전환할 모드는 모듈 생성 시 미리 부착해야 한다. 기본 CLI는 둘 다 부착한다.
전역 Depth Sensor 후처리 설정을 켜지 않고 DSD RenderProduct에만 설정한다.
각 카메라의 렌더링 시간 정보도 보존한다. 같은 app.update 이후 읽는다는 것만으로
센서 버퍼의 완전한 동기화를 증명하지 않으므로 최초 실행에서 annotator metadata를 확인한다.

DSD는 `rgb` annotator를 지원하지 않는다. DSD의 RGB는 같은 카메라 prim과 광학 설정을 공유하는
별도 표준 `CameraSensor` RenderProduct에서 수집한다. 따라서 both 모드는 RenderProduct 세 개를 사용한다.
RGB RenderProduct 경로와 annotator metadata도 저장하며, RGB/DSD 시간 정합은 실제 출력으로 검증해야 한다.

반환 `SensorFrame`: `depth_m`, `valid`, `rgb`, `points_native`, `metadata`.
DSD를 DIP와 같은 단순 기하 투영으로 바꾸지 않는다. DSD는 공식
`depth_sensor_point_cloud_position`을 쓰되, 렌더러의 `near**2` 배율만 나눈다(검증 기록 참고).
이는 단위 보정이며 좌표계 변환이 아니다. `dsd_depth_scale`로 원본 값을 복원할 수 있다.
이 좌표는 native left imager 기준이며
광축/거리 의미, 축 방향, baseline에 따른 원점 차이를 실제 출력으로 먼저 검증해야 한다.
`valid`는 유한값·양수·범위 조건만 검사한다. **정확성 신뢰도나 표면 검출 성공 라벨이 아니다.**

DSD 설정은 baseline(mm), focal length(pixel; K에서 계산), sensor width(pixel), 최대 disparity,
confidence threshold, disparity noise, outlier removal, 거리 제한을 노출한다.
이 모델은 실제 좌우 영상에 제조사 펌웨어를 실행하는 방식도, LiDAR 발광 모델도 아니다.
따라서 DSD의 성능 변화로 LiDAR의 투명 물체 성능을 주장할 수 없다.

## 실행

아래 명령은 프로젝트 디렉터리에서 실행한다. 현재 확인된 Isaac Python은
`/home/airlab/env_isaaclab/bin/python`이다. 다른 PC에서는 Isaac 6.0.1의 Python으로 바꾼다.

```bash
/home/airlab/env_isaaclab/bin/python transparent_sensor_test/run.py --preflight
/home/airlab/env_isaaclab/bin/python transparent_sensor_test/run.py --mode both --smoke --stay-open
/home/airlab/env_isaaclab/bin/python transparent_sensor_test/run.py --mode dip --headless
/home/airlab/env_isaaclab/bin/python transparent_sensor_test/run.py --mode dsd --headless
/home/airlab/env_isaaclab/bin/python transparent_sensor_test/run.py --mode both --headless
```

- 최초에는 GUI smoke를 사용한다. 0.8 m/0°에서 센서마다 한 번씩 저장한다(dip/dsd/lidar 3개).
- DSD 또는 DIP만 선택할 때도 동일한 장면 생성 코드를 사용한다.
- **한 번의 스캔에 네 컵이 모두 들어간다.** 센서는 네 컵 중심의 평균점을 겨냥한다.
  조건별로 따로 촬영하지 않으므로 `--target`은 없어졌다.
- `--no-lidar`로 RTX Lidar를 빼면 카메라 출력만 저장한다.
- `--config`로 다른 JSON을 사용하고 `--output`으로 **아직 없는** 출력 폴더를 지정한다.
- 출력은 기본적으로 이 폴더의 `runs/<UTC timestamp>/`이다.
- 기본적으로 시작 시 터미널에서 메모를 입력받는다. Enter만 누르거나 공백만 입력하면 `comment.md`를 만들지 않는다.
  `--comment "실험 목적 또는 변경 사항"`으로 메모를 미리 지정할 수 있으며, `--comment ""`는 질문 없이 건너뛴다.
  입력 스트림이 닫힌 경우에도 메모 없이 진행한다. `--ask-comment`는 기존 명령 호환용으로 유지한다.
  메모는 Isaac Sim 시작 전에 각 run 폴더의 `comment.md`에 UTF-8 Markdown으로 저장된다.
  preflight/설정 검사만 실행할 때는 질문하거나 run 폴더를 만들지 않는다.
- Isaac 에셋 서버 접근이 안 되면 `config.json`의 `asset_root`를 로컬 에셋 루트로 설정한다.
  루트 아래 `Isaac/Environments/Simple_Room/simple_room.usd`가 있어야 한다.
- GPU 또는 버전 점검 실패 시 렌더링을 시작하지 않고 exit 2로 끝난다.
- `--validate-config`와 기하 테스트는 GPU 없이 실행 가능하다.

### 출력

- `comment.md`: 내용이 있는 경우에만 저장하는 실험 메모. 이후 실행이 실패해도 보존한다.
- `scene.usda`: 센서 생성 전, 네 컵과 재질을 설정하고 다시 불러온 장면.
- `scene_with_sensors.usda`: 센서 설정까지 포함한 완료 시점의 장면.
- `run.json`: 실험 설정, 에셋 URL, 카메라 실제 속성, 해석상 제한.
- 자세별 `*_scene_dip.npz` / `*_scene_dsd.npz`: depth(m), valid mask, RGB, native points,
  `reference_depth_m`(DIP 기준값).
- 자세별 `*_scene_lidar.npz`: `points_m`(월드 XYZ), `range_m`, `azimuth_deg`, `elevation_deg`,
  `intensity`, `material_id`, `object_id`, `echo_id`, `valid`.
- 자세별 `*.json`: 거리/각도/표본, 화면 안의 조건 목록, K, optical-to-world, annotator 정보.
- `status.json`: complete/failed. failed인 데이터는 전체 실험 완료 결과로 집계하지 않는다.
  `complete`는 저장 절차 완료를 뜻한다. `empty_depth_frames`와 `data_quality`도 확인한다.
  유효 깊이가 없는 프레임은 경고를 출력하고 `needs_review`로 표시하며 원본 배열은 보존한다.
  `--stay-open`에서도 캡처 완료 즉시 status를 저장한다.

`reference_depth_m`은 **같은 자세·같은 업데이트에서 읽은 DIP 출력**이다(이전의 `reference_target_z_m`을 대체).
컵 메시는 해석적인 광선 교차값이 없어서 기준을 DIP로 바꿨다. 판 실험에서 DIP는 세 재질 모두
박스 기준값과 0.0 mm로 일치했고 재질과 무관하게 표면을 돌려줬다.
**독립적인 기하가 아니라 센서 출력이므로 DIP 자신의 오차를 그대로 안고 있다.**
특히 유리·물처럼 굴절이 있는 경우 DIP가 무엇을 반환하는지 먼저 확인해야 한다.
`--mode dsd`만 실행하면 DIP가 없어 기준값이 NaN이며 `reference_source`에 그 사실을 기록한다.
DSD native depth의 축 의미를 검증하기 전에는 이 값과 직접 MAE를 계산하지 않는다.
`geometry.target_box_depth`는 판 전용 해석 기준값으로 남아 있고 단위 테스트가 유지하지만
컵 표적에는 쓰지 않는다. 깊이 파이프라인을 다시 검증할 때 쓸 수 있다.

## 최초 GPU 실행에서 확인할 순서

1. Simple Room과 컵 에셋의 USD/텍스처 참조가 모두 로딩되고 네 조건이 구분되는지 확인.
   물 실린더가 컵 벽을 뚫지 않는지, 수면 높이가 그럴듯한지도 같이 본다.
2. 네 컵이 모두 화면 안에 있고 기존 방 가구에 가려지지 않는지 확인.
3. 불투명 컵 정면에서 DIP 중심 깊이를 컵 앞면까지 거리와 비교(거리 − 몸통 반지름).
4. 같은 깊이의 off-axis 픽셀에서 DSD의 native points와 depth를 비교하여
   axial Z인지 radial range인지, native left-imager 축/원점이 무엇인지 확인.
5. 동일 프레임의 DSD/DIP 해상도, 데이터 갱신, RGB와 depth의 시점 정합 확인.
6. 유리/알파 투명에서 결측·배경 측정·표면 측정의 차이를 관찰.
7. 재질 효과가 확인된 경우에만 거리·각도 sweep. 결측이 없으면 모델의 관측 결과로 기록한다.
8. 재질 배치 순환과 DSD noise=0 비교를 추가하여 위치·확률적 노이즈의 영향을 분리.

기본 sweep은 4거리 × 4각도 × 3표본이고, 한 프레임에 네 조건이 모두 들어간다.
**0.6 m에서는 바깥쪽 두 컵이 화면 밖으로 잘린다.** 네 컵이 모두 들어가려면 0.79 m 이상이어야 하고,
0.8 m에서도 여유가 6 mm뿐이다. run.py가 시작 시 이 조건을 계산해 경고한다.
컵은 회전 대칭에 가깝고 손잡이만 비대칭이므로, 판에서와 달리 각도 sweep의 의미가 약해진다.
손잡이 방향을 바꾸는 편이 입사각 변화를 더 잘 만든다.
both에서는 두 방식의 결과가 각각 저장된다. 물체/센서의 위치를 바꾸면 30프레임 안정화하고
샘플 사이에는 3프레임 진행한다. 안정화 횟수와 실제 버퍼 갱신을 첫 실행에서 검증한다.
일반 잡음이 줄었다는 사실과 투명 재질 오류가 줄었다는 사실을 구분해야 한다.

## Physics Raycast 확장 평가

로컬 6.0.1에 `isaacsim.sensors.experimental.physics.Raycast` / `RaycastSensor`가 있다.
레이별 원점, 방향, 시간 오프셋, 최소/최대 거리, SENSOR/WORLD 출력 프레임,
hit prim path를 지정할 수 있다. 출력은 거리, 충돌 위치, 법선 등이다.

추천 용도는 (a) 주변 가림까지 포함한 기하학적 기준값,
(b) 균일 그리드/관심영역 집중/회전 스캔 패턴 비교,
(c) 로봇 움직임과 순차 발사 시간의 영향 분석이다.

**충돌 형상에 대한 교차 계산이므로 광학적인 상위 모델은 아니다.**
같은 Collider를 가진 네 컵은 기본 Raycast에서 같은 표면으로 잡히는 것이 정상이다.
반사, 굴절, 파장, 수신 임계값, 다중 경로를 원한다면 RTX LiDAR/Acoustic/Radar 또는
별도 물리 응답 모델이 필요하다. 표면 법선으로 입사각 의존 검출 확률을 넣는 개발은 가능하지만
실측으로 보정하기 전에는 '사용자가 가정한 응답 모델'로 명시해야 한다.

다음 어댑터는 `RaycastGeometryBackend`로 별도 추가하는 편이 좋다.
반환값은 range + hit mask + origins/directions + points + normals + timestamp로 정의한다.
miss가 maxRange로 반환되는 규칙을 처리해야 하며, maxRange를 장애물 점으로 만들면 안 된다.
픽셀 그리드와 동일한 방향을 쓰더라도 range를 DIP Z로 바꾸는 변환이 필요하다.
시간 오프셋을 사용하면 센서가 움직이는 동안 각 광선의 시각별 자세도 적용해야 한다.

## 초음파: 있음

설치된 6.0.1 소스에서 확인:

- `isaacsim.sensors.experimental.rtx.Acoustic`
- `isaacsim.sensors.experimental.rtx.AcousticSensor`
- USD prim: `OmniAcoustic`, schema: `OmniSensorGenericAcousticWpmAPI`.
- `omni:sensor:WpmAcoustic:centerFrequency` 설정; 로컬 테스트는 40,000 Hz도 사용.
- sensor mount / receiver group 설정 지원.
- 출력: GenericModelOutput. raw acoustic 출력을 곧바로 단일 거리 값으로 가정하지 않는다.
- `SUPPORTED_ACOUSTIC_CONFIGS`는 현재 설치본에서 빈 dict이다.
  **프레임워크는 있지만 등록된 OEM 프리셋은 없다는 의미이며, 초음파가 없다는 의미는 아니다.**

다음 단계: 단일 송수신 배치 → 불투명 평판에서 왕복시간/거리 의미 검증 → 유리/각도/거리 변화.
빔 방향성, 주파수, 송수신 배열, 수신 임계값, 샘플링/갱신 주기 등 실제 스키마의 지원 항목을
확인하고 기록한다. Acoustic 클래스만 생성했다고 특정 초음파 제품의 동작이 검증되는 것은 아니다.
DSD/DIP 결과를 보완할 때 넓은 음향 빔의 거리 제약을 정확한 픽셀 한 개로 취급하지 않는다.

## IWRL6432AOP: 공식 에셋 등록도 확인

로컬 `SUPPORTED_RADAR_CONFIGS`에 다음 에셋이 있다:

`/Isaac/Sensors/TexasInstruments/IWRL6432AOP/IWRL6432AOP.usd`

따라서 나중에 `Radar.create(config="IWRL6432AOP", ...)`와 `RadarSensor` 어댑터로
추가하는 방향을 검토할 수 있다. **에셋 경로 등록을 확인했으며 다운로드/실행은 아직 검증하지 않았다.**
실제 칩은 57–63.5 GHz FMCW 레이더, 2 TX / 3 RX, antenna-on-package이다.
초음파와는 다른 측정 원리이다. 일반적으로 레이더가 유리/플라스틱을 투과할 수 있으므로
'카메라에 투명한 물체를 레이더가 반드시 검출한다'는 가정은 성립하지 않는다.
특히 정적 장애물 실험에서는 static clutter 제거가 목표 물체까지 지우는지 확인해야 한다.

유리/아크릴 판, 빈 컵, 물이 든 컵, 불투명 대조군에 대해 표면/배경/결측을 분리하고,
최소거리, 실제 chirp bandwidth, 거리·각도 분해능, 수신 패턴, SNR/탐지 임계값,
정적 clutter 처리, 센서 장착 원점 및 축을 평가한다.
시뮬레이터 에셋이 TI ADC/FFT/CFAR/펌웨어 전체를 동일하게 구현한다고 가정하지 않는다.
카메라처럼 조밀한 depth map으로 강제 변환하기보다 희소 거리/방위 관측으로 보존한다.

권장 순서: DSD/DIP 대조군 → Physics Raycast 기준값/주사패턴 → RTX Acoustic 보완 실험 →
IWRL6432AOP 비교 실험. 물리 원리가 달라지는 확장은 '센서 교체'와 '보완 관측 추가'로 구분한다.
현재 구현에는 DSD/DIP만 포함되며 Raycast/Acoustic/Radar 어댑터는 이 평가에 따른 후속 작업이다.

## 검증 기록

- 2026-09-14 DSD 전부 0의 실제 원인: **카메라 near 클리핑 평면이다.** GPU에서 이분 탐색으로 확인했다.
  6.0.1의 depth sensor 후처리는 깊이를 미터가 아니라 `실제 거리 / near**2`로 출력하고,
  min/max distance 컬링도 그 부풀려진 값에 적용한다. `near_m=0.05`에서는 배율이 400배이므로
  0.797 m 판이 318.8로 나오고 `max_distance_m=10`을 넘겨 전 픽셀이 컬링되어 정확히 0이 된다.
  측정으로 확인한 관계식(near 0.26/0.30/0.40/0.50 × 거리 0.6/0.8/1.2 m에서 소수점 4자리까지 일치):
  `보고값 = 실제 거리 / near**2`. near 기본값 1.0에서만 배율이 1이라 그대로 맞는 것처럼 보인다.
  far 평면도 작으면(10 m) 같은 출력을 비선형으로 왜곡하므로 렌더링 far는 크게 두어야 한다.
  수정: 렌더링 clipping range는 `(near_m, 1e6)`, DSD 거리 컷오프는 `near**2`로 나눠서 지정,
  출력 depth와 point cloud는 `near**2`를 곱해 미터로 되돌린다. `dsd_depth_scale`을 메타데이터에 남겨
  원본 값을 복원할 수 있다. `config.json`의 `far_m`은 이제 렌더링 클립이 아니라 유효성 판정 기준이다.
- 위 수정으로 `--mode both --smoke`를 GPU에서 재실행해 검증했다(`20260914T092625_327592Z`).
  DSD 유효 픽셀 0% → 68.5%, 중심 깊이는 세 재질 모두 0.7970 m로 박스 교차 기준값과 **오차 0.0 mm**이다.
  point cloud Z도 같은 배율로 부풀려져 있어 동일하게 보정하면 0.7970 m로 일치한다.
  세 재질의 DSD 결과는 서로 다르다(유효 마스크 약 12.5만 픽셀 차이). `empty_depth_frames`는 비었다.
- 기각된 가설: `DepthSensor: Texture sizes do not match`(DLSS 절반 해상도 depth 입력)는 원인이 **아니다.**
  공식 최소 장면에서 같은 경고가 뜬 상태로도 유효 깊이가 52% 나왔고, 경고를 없앤 우리 장면은 여전히 0%였다.
  각 RenderProduct의 `omni:rtx:post:aa:op=none` 설정은 depth 입력 해상도를 지키는 의미만 있어 유지한다.
  confidence/baseline/focal/sensor size/max disparity/해상도도 원인이 아니다.
  공식 최소 장면에 우리 값 전체를 그대로 넣어도 640×480에서 59.3% 유효였다.
- 2026-09-15 표적을 판에서 컵(`SM_Mug_A2`)으로 교체하고 `--mode both --smoke`로 검증했다
  (`20260915T014609_844112Z`). 네 조건 모두 저장되고 `empty_depth_frames`는 비었다.
  0.8 m에서 DIP 100%, DSD 약 67.5% 유효이고 중심 깊이는 네 조건 모두 0.7540 m로
  0.8 m − 몸통 반지름 0.046 m와 일치한다. DSD 깊이 영상에서 손잡이까지 형태가 분리된다.
  네 조건의 RGB와 DSD 유효 마스크는 서로 다르다(약 12.7만 픽셀 차이).
  빈 유리컵과 물 채운 컵도 컵 영역에서 평균 절대차 80으로 구분된다.
  에셋 RootNode가 자체 xformOp를 갖고 있어 슬롯 Xform과 참조 prim을 분리해야 했다.
  VRAM이 1.4 GB만 남았을 때는 `Out of resource descriptors!`로 실패했다. 3 GB 이상 확보하고 실행한다.
- 2026-09-15 조건별 촬영을 장면 단위 스캔으로 바꾸고 RTX Lidar를 세 번째 센서로 붙였다.
  `--mode both --smoke`로 검증했다(`20260915T030430_413403Z`). 자세마다
  `*_scene_dip.npz` / `*_scene_dsd.npz` / `*_scene_lidar.npz` 3개가 나오고 `empty_depth_frames`는 비었다.
  Lidar는 446,534개 반환(전부 valid, scanComplete=1), range 0.452~6.537 m였다.
  **컵 중심 9 cm 안의 `matId`가 재질과 정확히 일치했다**: opaque/transparent는 11(plastic),
  omniglass/omniglass_water는 19(clear_glass). 즉 카메라 깊이 annotator와 달리
  Lidar는 유리와 플라스틱을 구분한다. 다만 네 컵의 반환 수는 2057~2312로 비슷해
  **이 기본 Lidar 설정에서 유리 때문에 반환이 빠지는 현상은 관찰되지 않았다.**
  GMO 출력은 `FrameOfReference`와 무관하게 항상 구면 좌표였고, 단위는 도(degree)였다.
- 2026-09-15 Lidar 점군이 반원판처럼 보이는 문제 진단: **좌표 변환 버그가 아니었다.**
  모든 점이 기록된 센서 원점 하나에서 나오며 `|‖p-o‖ - range|`의 최대값이 8.5e-07 m였다.
  원인은 장착 높이였다. 센서가 컵 몸통 중심(1.2455 m)을 따라가면서 상판(1.200 m) 위 45.5 mm에 놓였고,
  아래쪽 링이 상판을 스치면서 상판 끝에서 잘린 거대한 동심원 호를 만들었다.
  장착 높이를 0/0.15/0.25/0.40 m로 비교한 결과 0.25 m(아래로 17.4° 기울어짐)에서
  상판 샘플이 29,513 → 82,019점으로 늘고 네 컵이 각각 뚜렷한 링으로 분리됐다.
  `lidar.mount_height_m`으로 조정한다. 검증 실행은 `verify_lidar_mount`이다.
  같은 조사에서 확인한 사실: 반환의 87.8 %가 matId 0(방)이고, 안쪽 두 컵이 바깥쪽보다
  반환이 많은 것은 재질이 아니라 보어사이트 기하 때문이다.
- 2026-09-15 "Lidar에 아무것도 안 잡힌다"의 원인: **`farRangeM` 기본값 200 m.**
  점군에는 컵이 처음부터 들어 있었다. 실험대 상판보다 위(z>1.21)이고 각 슬롯 축 5 cm 안인
  점만 세면 컵마다 1707~2108점이었고 matId도 정확했다(opaque/transparent=11, 유리 둘=19).
  다만 그것이 전체 460,854점의 1.8 %뿐이라, 방 전체를 스캔한 바닥 링에 묻혀 보이지 않았다.
  `far_range_m`을 2.0 m로 자르자 실험대와 네 컵이 눈으로 바로 보인다.
  사거리별 비교(마운트 0.25 m): 200 m → 460,861점/방 80.5 %, 3 m → 132,896점/35.4 %,
  2 m → 128,418점/34.5 %, 1.5 m → 106,353점/23.3 %.
  검증 실행은 `verify_lidar_range`(128,406점, 컵 4,451점, 상판 79,711점)이다.
- 5개 기하 테스트 통과: 좌표계 handedness, 앞면 두께 보정, 경사각, axial/radial 차이,
  월드 평행이동 불변성, 카메라 뒤쪽 물체 배제.
- 기본 설정 검사 통과.
- Python 구문 검사와 사용한 주요 API에 대한 로컬 소스 대조.
- GPU preflight 실패: NVIDIA-SMI could not communicate with the NVIDIA driver.
  이 결과만으로 호스트 드라이버의 설치 상태나 문제 원인을 단정하지 않는다.
- 렌더링/에셋/실측 성능 검증 미완료.

## 근거 자료

- [6.0.1 Camera Sensors](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_camera.html)
- [6.0.1 Depth Sensors](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_camera_depth.html)
- [Single View Depth 알고리즘과 출력](https://docs.omniverse.nvidia.com/kit/docs/omni.sensors.nv.camera/1.0.0/index.html)
- [6.0.1 환경 에셋](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/assets/usd_assets_environments.html)
- [센서용 재질](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx_materials.html)
- [Physics Raycast 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_raycast.html)
- [RTX Acoustic 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_acoustic.html)
- [6.0.0 릴리스의 Acoustic·IWRL6432AOP 지원](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html)
- [공식 IWRL6432AOP 에셋 목록](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_nonvisual_sensors.html)
- [TI IWRL6432AOP 사양](https://www.ti.com/product/IWRL6432AOP)
- [TI mmWave Radome Design Guide](https://www.ti.com/lit/an/swra705/swra705.pdf)

latest 링크는 변경될 수 있다. 이 문서의 6.0.1 지원 판단은 6.0.x 공식 자료와 로컬 설치본 소스를 함께 대조했다.
