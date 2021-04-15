
# ARUco Detector 구현 프로젝트

## 목표
ARUco의 기준 마커(Fiducial marker)를 검출하고, 3차원 좌표계와 마커 아이디 번호를 마커 위에 출력하는 것입니다.

## ARUco란?
증강현실(Augmented Reality)에서 가상의 3차원 물체를 렌더링 할 때 사용하는 기준 마커(Fiducial marker)를 생성하고 검출하는 대표적인 방법입니다.   
현실 공간에서 가상의 3차원 물체를 렌더링 할 때는 어느 위치에서 어떤 자세로 표현될지에 대한 기준점이 필요합니다. 
기준을 정하는 방법은 크게 세 가지로, **GPS 좌표**를 기준으로 삼는 방법, 영상속 **실세계 객체**를 기준으로 삼는 방법, 그리고 미리 정한 **기준 마커**(Fiducial marker)를 기준으로 삼는 방법이 있습니다.  
그 중 **기준 마커**를 사용한 유명한 방법중 하나가 S. Garrido-Jurado 등이 2014년 출판한 논문 *"Automatic generation and detection of highly reliable fiducial markers under occlusion"* 에서 제시한 방법이고, 이를 **ARUco**라 부릅니다. 여기서 Uco는 저자들의 소속 대학인 University of Córdoba 의 약자입니다.

## 해당 기법의 의의
해당 기법의 핵심 아이디어는, 사전에 있는 기준 마커들 간의 오차의 최소값을 이론상 한계치에 근접하도록 키운 것입니다. 
오차의 최소값이 크면 마커가 일부 가려지거나 훼손되어도 대응하는 번호를 쉽가 찾을 수 있습니다.  

이 기법은 주어진 마커에 대응하는 수열을 생성하고, 두 마커에간의 오차를 생성한 수열들 간의 해밍거리로 정의합니다.  
이 때 사전에 있는 마커들 간의 오차의 최소값을 T 라 하면, 영상처리를 통해 추출한 마커 후보와 사전에 저장된 한 마커 간의 최소 오차가 [(T-1)/2] 이하일 때 마커 후보를 해당 마커라고 판단합니다.  
실제 구현에선 좀 더 엄격한 판별을 위해 최대 허용 오차를 더 작게 조정할 수 있습니다.

## Detection 알고리즘 개요
1. 입력으로 들어온 마커가 포함된 컬러 영상을 흑백 영상으로 변환합니다. 
2. 변환한 흑백영상을 adpative thresholding 기법으로 이진화 합니다
3. 이진화한 영상에서 경계선을 계산한 뒤, Ramer–Douglas–Peucker 알고리즘으로 경계선들을 단순화 합니다. 단순화한 경계선들 중 볼록한 사각형 모양만 선별하여 그 코너점들을 저장합니다. 
4. 선별한 코너에 대응하는 마커 후보를 1번에서 나온 흑백영상에서 추출한 뒤, 개별 마커 영상을 Otsu의 알고리즘으로 이진화를 합니다. 이후 각 마커 영상을 bit matrix로 변환합니다.
5. 변환한 bit matrix와 일치하는 기준 객체를 사전에서 찾아 오차를 최소화 하는 마커의 코너점을 출력합니다. 이 때 최소 오차가 기준치를 넘어서면 마커가 아니라 판별합니다.

## 프로젝트 구성
detector, calibrator, marker generator, 그리고 board generator 로 구성되어 있습니다. 직접 구현한 것은 detector 뿐이고, calibrator와 두 generator들은 라이브러리와 샘플 코드를 활용했습니다.
1. ""MyARUcoDetector""   : 직접 구현한 ARUco Marker detector 프로젝트 입니다. openCV 4.5.2 라이브러리가 필요합니다.
2. ARUcoCalibrator       : 카메라 캘리브레이션에 필요한 calibrator를 위한 프로젝트 입니다. 체커보드에 ARUco 마커를 합성한 Charuco board로 캘리브레이션을 하고, ARUco 와 openCV 4.5.2 라이브러리가 필요합니다.
3. ARUcoMarkerGenerator  : ARUco detection에 필요한 fiducial marekr와 프로젝트 입니다. ARUco 와 openCV 4.5.2 라이브러리가 필요합니다.
4. CharucoBoardGenerator : calibration에 필요한 Charuco를 생성하는 프로젝트 입니다.  ARUco 와 openCV 4.5.2 라이브러리가 필요합니다.
