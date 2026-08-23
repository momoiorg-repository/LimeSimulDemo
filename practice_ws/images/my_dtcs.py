import cv2  # openCVライブラリのインポート
import numpy as np  # numpyライブラリのインポート
from cv2 import aruco, imread, imwrite

##　↓↓↓↓↓↓↓inRangeWrap, calc_centroidは変更しないでください↓↓↓↓↓↓
# inRangeを色相が0付近や180付近の色へ対応する形へ修正
def inRangeWrap(hsv, lower, upper):
    if lower[0] <= upper[0]:
        return cv2.inRange(hsv, lower, upper)
    else:
        # 180をまたぐ場合
        lower1 = np.array([0, lower[1], lower[2]])
        upper1 = np.array([upper[0], upper[1], upper[2]])
        lower2 = lower
        upper2 = np.array([179, upper[1], upper[2]])
        return cv2.bitwise_or(
            cv2.inRange(hsv, lower1, upper1),
            cv2.inRange(hsv, lower2, upper2)
        )
    
def calc_centroid(mask):
    M = cv2.moments(mask)
    if M["m00"] != 0:
        # 重心座標を計算SS
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        s = np.count_nonzero(mask)/(mask.shape[0]*mask.shape[1])
        return cx, cy, s
    else:
        return None   
##　↑↑↑↑↑↑↑inRangeWrap, calc_centroidは変更しないでください↑↑↑↑↑↑↑

def d_ball(img):
    # 画像の読み込み
    draw_img = img.copy() # 元データを書き換えないようにコピーを作成
    # HSVに変換（色指定はRGBよりHSVの方が扱いやすい）
    hsv_img = cv2.cvtColor(draw_img, cv2.COLOR_BGR2HSV)

    # BGR空間での抽出範囲
    ## ボール
    lower = np.array([0, 220, 170]) # 色相, 彩度, 明度 の下限
    upper = np.array([10, 240, 255]) # 色相, 彩度, 明度 の上限

    # 指定範囲に入る画素を抽出（白が該当部分）
    mask = inRangeWrap(hsv_img, lower, upper)
    
    try:
        x, y, s = calc_centroid(mask)
        print(f"{s=}")
        return x, y
    except TypeError:
        return None

def d_coke(img):
    # 画像の読み込み
    draw_img = img.copy() # 元データを書き換えないようにコピーを作成
    # HSVに変換（色指定はRGBよりHSVの方が扱いやすい）
    hsv_img = cv2.cvtColor(draw_img, cv2.COLOR_BGR2HSV)

    # BGR空間での抽出範囲
    ## コーラ缶
    lower = np.array([165, 150, 0]) # 色相, 彩度, 明度 の下限
    upper = np.array([180, 255, 255]) # 色相, 彩度, 明度 の上限

    # 指定範囲に入る画素を抽出（白が該当部分）
    mask = inRangeWrap(hsv_img, lower, upper)
    
    try:
        x, y, s = calc_centroid(mask)
        print(f"{s=}")
        return x, y
    except TypeError:
        return None

def d_circle(img):
    # 画像読み込み
    draw_img = img.copy()

    # 前処理（グレースケール＋ぼかし）
    gray = cv2.cvtColor(draw_img, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)  # ノイズ低減

    # 円検出（HoughCircles）
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT,
        dp=1.2, minDist=30,
        param1=100, param2=50,   
        minRadius=10, maxRadius=40  
    )
    # マスク作成（検出円を塗りつぶし）
    mask = np.zeros(gray.shape, dtype=np.uint8)
    if circles is not None:
        circles = np.round(circles[0]).astype(int)
        for x, y, r in circles:
            cv2.circle(mask, (x, y), r, 255, -1)     # マスク（白塗り）
            break

    x, y, s = calc_centroid(mask)
    print(f"{s=}")
    if x and y:
        return x, y
    else:
        return None

