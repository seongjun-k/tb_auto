#!/usr/bin/env python3
"""dataset_raw/{class}/*.jpg + 같은 이름의 *.txt(labelImg YOLO 출력) -> dataset/ 8:2 분할."""
import glob
import os
import random
import shutil

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]
ROOT = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(ROOT, "dataset_raw")
LABELS = os.path.join(RAW, "labels")   # labelImg에 save_dir를 줬을 때 txt가 모이는 곳
OUT = os.path.join(ROOT, "dataset")
VAL_RATIO = 0.2


def label_of(jpg):
    for txt in (jpg[:-4] + ".txt", os.path.join(LABELS, os.path.basename(jpg)[:-4] + ".txt")):
        if os.path.exists(txt):
            return txt
    return None


def main():
    random.seed(0)  # 재실행해도 같은 분할
    pairs, unlabeled = [], []
    for jpg in sorted(glob.glob(os.path.join(RAW, "*", "*.jpg"))):
        # 이미지 옆 / 공용 labels 디렉토리 둘 다 본다 (labelImg 저장위치 설정에 따라 갈림)
        txt = label_of(jpg)
        (pairs if txt else unlabeled).append(jpg)

    if unlabeled:
        print("라벨 없음 %d장 (제외): %s ..." % (len(unlabeled), os.path.basename(unlabeled[0])))
    if not pairs:
        raise SystemExit("라벨된 이미지가 없다. labelImg로 먼저 라벨링할 것.")

    # 라벨 인덱스가 소속 폴더와 다르면 labelImg에서 클래스를 잘못 고른 것.
    # 400장 중 몇 장 틀린 건 눈으로 못 잡는다. 여기서 걸러야 학습을 헛돌리지 않는다.
    bad = []
    for jpg in pairs:
        want = CLASSES.index(os.path.basename(os.path.dirname(jpg)))
        with open(label_of(jpg)) as f:
            got = {int(ln.split()[0]) for ln in f if ln.strip()}
        if got != {want}:
            bad.append((jpg, sorted(got), want))
    if bad:
        print("!! 폴더와 클래스 인덱스 불일치 %d건 (labelImg에서 클래스 잘못 고름):" % len(bad))
        for jpg, got, want in bad[:10]:
            print("   %s  라벨=%s  폴더기준=%d(%s)" % (os.path.relpath(jpg, RAW), got, want, CLASSES[want]))
        raise SystemExit("고치고 다시 실행할 것. (의도한 것이면 이 검사를 지울 것)")

    random.shuffle(pairs)
    n_val = int(len(pairs) * VAL_RATIO)
    splits = {"val": pairs[:n_val], "train": pairs[n_val:]}

    for sub in ("images", "labels"):
        for split in splits:
            d = os.path.join(OUT, sub, split)
            shutil.rmtree(d, ignore_errors=True)
            os.makedirs(d)

    for split, files in splits.items():
        for jpg in files:
            base = os.path.basename(jpg)
            shutil.copy(jpg, os.path.join(OUT, "images", split, base))
            shutil.copy(label_of(jpg), os.path.join(OUT, "labels", split, base[:-4] + ".txt"))

    with open(os.path.join(OUT, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\n\nnc: %d\nnames:\n" % (OUT, len(CLASSES)))
        f.writelines("  %d: %s\n" % (i, c) for i, c in enumerate(CLASSES))

    print("train=%d val=%d -> %s" % (len(splits["train"]), len(splits["val"]), OUT))


if __name__ == "__main__":
    main()
