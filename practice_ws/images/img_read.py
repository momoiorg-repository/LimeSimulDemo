import cv2  # openCVライブラリのインポート

# 画像の読み込み
img = cv2.imread("./imgs/sample1.png")

# 画像の表示
cv2.imshow("sample",img)
cv2.waitKey(0)  # 何かのキーが押されるまでウィンドウを表示