import cv2  # openCVライブラリのインポート
import numpy as np  # numpyライブラリのインポート

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
    
#######################################################################

def ball_detector(img):
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

def coke_detector(img):
    # 画像の読み込み
    draw_img = img.copy() # 元データを書き換えないようにコピーを作成
    # HSVに変換（色指定はRGBよりHSVの方が扱いやすい）
    hsv_img = cv2.cvtColor(draw_img, cv2.COLOR_BGR2HSV)

    # BGR空間での抽出範囲
    ## コーラ缶
    lower = np.array([170, 230, 0]) # 色相, 彩度, 明度 の下限
    upper = np.array([180,255,255]) # 色相, 彩度, 明度 の上限

    # 指定範囲に入る画素を抽出（白が該当部分）
    mask = inRangeWrap(hsv_img, lower, upper)
    
    try:
        x, y, s = calc_centroid(mask)
        print(f"{s=}")
        return x, y
    except TypeError:
        return None

# # 結果表示
# cv2.imshow("Original + Centroid", draw_img)
# cv2.imshow("Mask", mask)
# cv2.imshow("Result", result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# cv2.imwrite("./imgs/output3.png", result)
# cv2.imwrite("./imgs/mask.png", mask)

def test_func():
    print("test")