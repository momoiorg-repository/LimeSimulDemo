import cv2  # openCVライブラリのインポート

# 色取得関数
def get_color(event, x, y, flags, param):
    img=param
    if event == cv2.EVENT_LBUTTONDOWN:
        b, g, r = img[y, x]
        print(f"座標({x}, {y}) の色：B={b}, G={g}, R={r}")

# 画像の読み込み
img = cv2.imread("./imgs/sample1.png")

# ウィンドウを表示してコールバック登録
cv2.imshow("image", img)
cv2.setMouseCallback("image", get_color, param=img)

cv2.waitKey(0)