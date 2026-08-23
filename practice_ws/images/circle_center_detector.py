import cv2
import numpy as np

# 画像読み込み
img = cv2.imread("./imgs/sample1.png")
draw_img = img.copy()

# 前処理（グレースケール＋ぼかし）
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.medianBlur(gray, 5)  # ノイズ低減

# 円検出（HoughCircles）
circles = cv2.HoughCircles(
    gray, cv2.HOUGH_GRADIENT,
    dp=1.2, minDist=30,
    param1=100, param2=50,   # 感度はparam2で調整
    minRadius=10, maxRadius=40  
    # 半径を調整することで、接近距離も調整できる。
    # ただし、param2が大きいと、半径の範囲が収まっていても検出されない場合があるので注意。
)

# マスク作成（検出円を塗りつぶし）
mask = np.zeros(gray.shape, dtype=np.uint8)
if circles is not None:
    circles = np.round(circles[0]).astype(int)
    for x, y, r in circles:
        cv2.circle(mask, (x, y), r, 255, -1)     # マスク（白塗り）
        cv2.circle(draw_img, (x, y), r, (0, 255, 255), 2)  # 参考表示
        print(f"円の半径: {r} [px]")
        print(f"円の面積: {int(mask.sum())//255}/{mask.shape[0]*mask.shape[1]}[px]")

# 重心計算（マスクに対して）
M = cv2.moments(mask)
if M["m00"] != 0:
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    cv2.drawMarker(draw_img, (cx, cy), (0, 255, 0),
                   markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    print(f"重心座標：x={cx}, y={cy}")
else:
    print("重心が計算できません（マスクが空です）．")

# # 結果表示・保存
cv2.imshow("Detected Circles + Centroid", draw_img)
cv2.imshow("Mask (from circles)", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

# cv2.imwrite("./imgs/circle_mask.png", mask)
# cv2.imwrite("./imgs/circles_with_centroid.png", draw_img)
