#!/usr/bin/env python3
"""폴더명으로 클래스를 알고 있으니 위치만 찾으면 된다 -> 색으로 박스 자동 생성.

라벨 규칙 (사람이 찍은 것과 동일하게 맞춤):
  표지판: 파란 원에 딱 맞게
  신호등: 램프가 아니라 검은 하우징 전체

  python3 autolabel.py            # dataset_raw/labels/*.txt 생성 (기존 사람 라벨은 안 건드림)
  python3 autolabel.py --eval     # 사람이 만든 xml과 IoU 비교만
"""
import glob
import os
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]
ROOT = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(ROOT, "dataset_raw")
LABELS = os.path.join(RAW, "labels")


def _largest(mask, min_area=150):
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[i, cv2.CC_STAT_AREA] < min_area:
        return None
    return i, lab, stats


def find_sign(bgr):
    """파란 원."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (95, 120, 60), (130, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    # 화면에 표지판이 둘 보일 때가 있다. 잘린 쪽을 고르면 안 되므로
    # 경계에 닿지 않은 후보 중 가장 큰 것을 먼저 찾고, 없을 때만 최대 블롭으로 떨어진다.
    ih, iw = m.shape
    n, lab, st_all, _ = cv2.connectedComponentsWithStats(m, 8)
    cands = [j for j in range(1, n) if st_all[j, cv2.CC_STAT_AREA] >= 400]
    inside = [j for j in cands
              if st_all[j, 0] > 1 and st_all[j, 1] > 1
              and st_all[j, 0] + st_all[j, 2] < iw - 1 and st_all[j, 1] + st_all[j, 3] < ih - 1]
    pool = inside or cands
    if not pool:
        return None
    i = max(pool, key=lambda j: st_all[j, cv2.CC_STAT_AREA])
    st = st_all
    x, y, w, h = st[i, 0], st[i, 1], st[i, 2], st[i, 3]
    # 파란 마스크가 사람 라벨보다 오른쪽으로 조금 번진다. 실측 보정 (n=5, std<0.05)
    sc = (w + h) / 2.0
    return (int(x + 0.029 * sc), int(y + 0.041 * sc),
            int(x + w - 0.093 * sc), int(y + h + 0.010 * sc))


# 램프 bbox -> 하우징 bbox 변환계수. 사람이 찍은 라벨에서 실측한 값이며
# 램프 크기 s=(w+h)/2 배수로 표현해 거리에 무관하다.
# red는 위쪽 램프라 아래로 길게(+1.71), green은 아래쪽 램프라 위로 길게 늘린다.
HOUSING = {
    "red_light": (-0.568, -0.353, +0.332, +1.710),
    "green_light": (-0.225, -1.171, +0.063, +0.018),   # 사람 라벨 98장에서 중앙값으로 실측
}


def find_light(bgr, cls):
    """켜진 램프를 찾아 하우징 박스로 확장한다.

    벽이 초록 램프와 색상(H)이 거의 같아서 H로는 못 가른다. 램프는 채도가 높고
    어둡고(S~200 V~77), 벽은 옅고 밝다(S~67 V~203). 검은 하우징은 램프보다 더 어둡다.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if cls == "green_light":
        lamp = cv2.inRange(hsv, (40, 150, 55), (95, 255, 125))
    else:
        lamp = cv2.inRange(hsv, (0, 110, 60), (10, 255, 255)) | \
               cv2.inRange(hsv, (170, 110, 60), (180, 255, 255))
    lamp = cv2.morphologyEx(lamp, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    lamp = cv2.morphologyEx(lamp, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    got = _largest(lamp, 100)
    if not got:
        return None
    i, _, st = got
    lx, ly, lw, lh = st[i, 0], st[i, 1], st[i, 2], st[i, 3]
    if not 0.5 < lw / float(lh) < 2.0:      # 램프는 원형. 길쭉하면 오검출
        return None
    sc = (lw + lh) / 2.0
    dl, dt, dr, db = HOUSING[cls]
    h, w = bgr.shape[:2]
    return (max(0, int(lx + dl * sc)), max(0, int(ly + dt * sc)),
            min(w, int(lx + lw + dr * sc)), min(h, int(ly + lh + db * sc)))


def detect(path, cls):
    bgr = cv2.imread(path)
    if cls in ("left_turn", "right_turn"):
        return find_sign(bgr), bgr.shape
    return find_light(bgr, cls), bgr.shape


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua else 0.0


def gt_boxes():
    out = {}
    # 사람이 찍은 PascalVOC 원본. labelImg에서 xml이 txt보다 우선순위라
    # 충돌을 피하려고 labels/ 밖으로 빼뒀다 (정확도 평가용으로는 계속 읽는다).
    for x in glob.glob(os.path.join(RAW, "labels_xml_backup", "*.xml")):
        r = ET.parse(x).getroot()
        o = r.find("object")
        if o is None:
            continue
        b = o.find("bndbox")
        out[r.findtext("filename")] = tuple(int(b.findtext(k)) for k in ("xmin", "ymin", "xmax", "ymax"))
    return out


def montage():
    """생성된 txt를 실제로 이미지에 그려서 클래스별 대조표를 만든다.
    빨강=자동 생성, 파랑=사람이 찍은 것."""
    gts = gt_boxes()
    out_dir = os.path.join(ROOT, "label_check")
    os.makedirs(out_dir, exist_ok=True)
    cols, tw, th = 10, 192, 144
    for cls in CLASSES:
        tiles = []
        for jpg in sorted(glob.glob(os.path.join(RAW, cls, "*.jpg"))):
            name = os.path.basename(jpg)
            txt = os.path.join(LABELS, name[:-4] + ".txt")
            if not os.path.exists(txt):
                continue
            img = cv2.imread(jpg)
            h, w = img.shape[:2]
            for ln in open(txt):
                _, xc, yc, bw, bh = (float(v) for v in ln.split())
                x1, y1 = int((xc - bw / 2) * w), int((yc - bh / 2) * h)
                x2, y2 = int((xc + bw / 2) * w), int((yc + bh / 2) * h)
                color = (255, 128, 0) if name in gts else (0, 0, 255)
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            tiles.append(cv2.resize(img, (tw, th)))
        if not tiles:
            continue
        while len(tiles) % cols:
            tiles.append(np.zeros((th, tw, 3), np.uint8))
        grid = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
        path = os.path.join(out_dir, cls + ".jpg")
        cv2.imwrite(path, grid)
        print("%-12s %3d장 -> %s" % (cls, sum(t.any() for t in tiles), path))


def main():
    if "--montage" in sys.argv:
        return montage()
    gts = gt_boxes()
    evaluate = "--eval" in sys.argv
    os.makedirs(LABELS, exist_ok=True)
    scores, missed, written = {}, [], 0

    for cls in CLASSES:
        scores[cls] = []
        for jpg in sorted(glob.glob(os.path.join(RAW, cls, "*.jpg"))):
            name = os.path.basename(jpg)
            box, shape = detect(jpg, cls)
            if box is None:
                missed.append(name)
                continue
            if name in gts:
                scores[cls].append(iou(box, gts[name]))
            if evaluate:
                continue
            # 사람이 찍은 라벨이 있으면 그걸 쓴다. 자동 박스로 덮어쓰지 않는다.
            x1, y1, x2, y2 = gts.get(name, box)
            h, w = shape[:2]
            with open(os.path.join(LABELS, name[:-4] + ".txt"), "w") as f:
                f.write("%d %.6f %.6f %.6f %.6f\n" % (
                    CLASSES.index(cls), (x1 + x2) / 2 / w, (y1 + y2) / 2 / h,
                    (x2 - x1) / w, (y2 - y1) / h))
            written += 1

    for cls in CLASSES:
        s = np.array(scores[cls])
        if len(s):
            print("%-12s 사람라벨 %3d장과 비교: IoU 평균 %.3f  >0.5 %d개  <0.5 %d개"
                  % (cls, len(s), s.mean(), (s > 0.5).sum(), (s <= 0.5).sum()))
    print("검출 실패 %d장" % len(missed))
    if missed[:5]:
        print("  " + ", ".join(missed[:5]))
    if not evaluate:
        print("txt %d개 생성 (사람 라벨 %d장은 xml 그대로 변환, 나머지는 자동)"
              % (written, len(gts)))


if __name__ == "__main__":
    main()
