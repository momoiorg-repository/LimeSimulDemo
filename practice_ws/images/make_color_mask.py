import cv2  # openCVライブラリのインポート
import numpy as np  # numpyライブラリのインポート

# 画像の読み込み
img = cv2.imread("./imgs/sample1.png")
draw_img = img.copy() # 元データを書き換えないようにコピーを作成

# BGR空間での抽出範囲
lower = np.array([0, 0, 0])     # B, G, R の下限
upper = np.array([0, 0, 0])     # B, G, R の上限

# 指定範囲に入る画素を抽出（白が該当部分）
mask = cv2.inRange(draw_img, lower, upper)

# マスクを適用して対象色のみ残す
result = cv2.bitwise_and(draw_img, draw_img, mask=mask)

# 画像表示
cv2.imshow("Original", draw_img)
cv2.imshow("Mask (BGR)", mask)
cv2.imshow("Result (BGR)", result)
cv2.waitKey(0)  # 何かのキーが押されるまでウィンドウを表示
