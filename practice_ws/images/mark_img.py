import cv2  # openCVライブラリのインポート

# 画像の読み込み
img = cv2.imread("./imgs/sample1.png")
draw_img = img.copy() # 元データを書き換えないようにコピーを作成

# ~ここを自作~

# 画像の表示と保存
cv2.imshow("sample",draw_img)
cv2.waitKey(0)  # 何かのキーが押されるまでウィンドウを表示
cv2.imwrite("./imgs/output1.png", draw_img)
