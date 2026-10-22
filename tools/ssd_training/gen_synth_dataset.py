#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render a synthetic training set for the task-card detector, with exact labels.

WHY SYNTHETIC FIRST
-------------------
The detector has to learn nine GB 2894 warning pictograms that we already have
as pixel-exact artwork (labels.py points at the same PNGs Gazebo textures the
cards with), on a mat whose colour and scale are known, seen by a camera whose
intrinsics and mounting are known too.  Generating that data is a rendering
problem, not a labelling problem - and the labels come out perfect for free,
which is the whole point: a hand-labelled set of the same size would take days
and still disagree with itself around the frame edges.

This set is the COLD START.  Real photographs are what close the sim-to-real
gap, and tools/ssd_training/README.md covers the fine-tuning pass.  What this
script must not do is teach the model something the aircraft will never see, or
hide something it will - hence the nuisance model below is deliberately built
out of the actual failure modes we have already met on this field:

  * the drone hovering off-centre, with heading error  (camera yaw/offset)
  * roll and pitch while it settles                    (perspective, not affine)
  * the altitude band the mission actually flies at    (scale)
  * JPEG at the quality the on-board link uses         (compression ringing)
  * raking light making a frame edge read weak         (brightness/contrast)
  * one card half out of frame                         (truncation, kept)

WHAT IT WRITES
--------------
  <out>/images/*.jpg          JPEG, 640x480, the camera's native size
  <out>/annotations/*.xml     Pascal VOC, which prepare_pytorch_voc.py consumes
  <out>/preview/*.png         annotated copies, for eyeballing the labels
  <out>/label_map.pbtxt       ids for the Object Detection API

Usage
-----
    # the default 4000 frames: about half carry the two hazard cards, which is
    # ~4000 hazard boxes over nine classes, plus ~4000 person boxes
    python3 tools/ssd_training/gen_synth_dataset.py --out /tmp/task_ds

    # look before you generate: 8 annotated previews, no dataset
    python3 tools/ssd_training/gen_synth_dataset.py --preview /tmp/prev

    # both zones mixed in every frame (harder, more clutter)
    python3 tools/ssd_training/gen_synth_dataset.py --out /tmp/task_ds --mixed
"""

import argparse
import math
import os
import random
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance, ImageFilter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import labels as L  # noqa: E402


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
class Camera(object):
    """A nadir camera on a hovering vehicle.

    The rotation is built from the vehicle's heading and then tilted, rather
    than being a plain scale-and-rotate, because the tilt IS the perspective:
    a 4-degree roll moves the far edge of a 0.45 m card by several pixels at
    this altitude, and an affine render would teach the model that cards stay
    perfect squares.  They do not, once the aircraft starts settling.
    """

    def __init__(self, position, yaw, pitch, roll, hfov, width, height):
        self.position = np.asarray(position, float)
        cy, sy = math.cos(yaw), math.sin(yaw)
        # Camera basis in world coordinates, nadir, heading = yaw:
        #   image +x (right) -> vehicle's right
        #   image +y (down)  -> behind the vehicle, so image up = nose
        #   optical axis     -> straight down
        base = np.array([[sy, -cy, 0.0],
                         [-cy, -sy, 0.0],
                         [0.0, 0.0, -1.0]])
        # Then tilt about the camera's own axes: roll about image-x, pitch
        # about image-y.  Small angles only; this is a settling vehicle, not a
        # gimbal.
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        tilt = np.array([[1.0, 0.0, 0.0],
                         [0.0, cr, -sr],
                         [0.0, sr, cr]]) @ \
               np.array([[cp, 0.0, sp],
                         [0.0, 1.0, 0.0],
                         [-sp, 0.0, cp]])
        self.rotation = tilt @ base
        self.width = width
        self.height = height
        self.fx = width / (2.0 * math.tan(hfov / 2.0))
        self.fy = self.fx
        self.cx = width / 2.0
        self.cy = height / 2.0

    def project(self, points_xy):
        """World points on the mat (z = 0) -> image pixels.

        Returns Nx2 floats.  A point behind the camera would divide by a
        non-positive depth; nothing in this scene can be behind a nadir camera
        tilted by only a few degrees, but the caller still drops such points
        rather than trusting that.
        """
        pts = np.asarray(points_xy, float).reshape(-1, 2)
        world = np.column_stack([pts, np.zeros(len(pts))])
        # world -> camera:  p_c = R (P - C)
        pc = (world - self.position) @ self.rotation.T
        depth = pc[:, 2]
        safe = np.where(np.abs(depth) < 1e-6, 1e-6, depth)
        u = self.fx * pc[:, 0] / safe + self.cx
        v = self.fy * pc[:, 1] / safe + self.cy
        return np.column_stack([u, v]), depth


def homography(src, dst):
    """3x3 H with dst ~ H @ src, from four point correspondences."""
    a = []
    for (x, y), (u, v) in zip(src, dst):
        a.append([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, -u])
        a.append([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, -v])
    _, _, vt = np.linalg.svd(np.asarray(a, float))
    return vt[-1].reshape(3, 3)


# ---------------------------------------------------------------------------
# Card artwork
# ---------------------------------------------------------------------------
_CARD_CACHE = {}


def compose_card(class_name, px):
    """One square card, RGBA, px pixels across, already including its frame.

    The frame is drawn here rather than left to the texture because in the world
    file it is four separate 0.025 m bars standing 2 mm proud - the textured
    0.40 m face does not contain it.  Composing the two the way the world does
    is what makes the synthetic card the same object the aircraft flies over.
    """
    key = (class_name, px)
    if key in _CARD_CACHE:
        return _CARD_CACHE[key]

    frame_px = max(2, int(round(px * L.FRAME_BAR_M / L.CARD_FRAME_M)))
    face_px = px - 2 * frame_px
    frame_rgb = L.FRAME_RED if L.zone_of(class_name) == "hazard" else L.FRAME_BLUE

    card = Image.new("RGBA", (px, px), frame_rgb + (255,))
    face = Image.open(L.texture_path(class_name)).convert("RGB")
    face = face.resize((face_px, face_px), Image.Resampling.LANCZOS)
    card.paste(face, (frame_px, frame_px))

    # A faint darker lip along two edges so the frame does not read as a flat
    # painted border - the synthetic equivalent of the 2 mm stand-off.
    draw = ImageDraw.Draw(card, "RGBA")
    shade = tuple(max(0, c - 55) for c in frame_rgb) + (110,)
    draw.line([(0, 0), (px - 1, 0)], fill=shade, width=max(1, frame_px // 3))
    draw.line([(0, 0), (0, px - 1)], fill=shade, width=max(1, frame_px // 3))

    _CARD_CACHE[key] = card
    return card


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
# Set from --box-extent before any rendering happens.  None means "the printed
# artwork" (per-class, from labels.artwork_box); a number means a square of that
# size, which is what the old behaviour was.
BOX_EXTENT_M = None


class Card(object):
    def __init__(self, class_name, x, y, yaw):
        self.class_name = class_name
        self.x = x
        self.y = y
        self.yaw = yaw

    def _place(self, local_pts):
        """Local mat-plane offsets (metres, card-centred) -> world metres."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + c * lx - s * ly, self.y + s * lx + c * ly)
                for lx, ly in local_pts]

    def card_corners(self):
        """The card as painted: 0.45 m square, frame bars included."""
        h = L.CARD_FRAME_M / 2.0
        return self._place([(-h, -h), (h, -h), (h, h), (-h, h)])

    def box_corners(self):
        """The card as annotated: the outline of the printed artwork.

        Three points for a hazard sign, four for the person, and not related to
        card_corners() by any single factor - so both are projected separately
        in render() rather than one being scaled into the other in image space,
        which would be right at the centre of the frame and wrong at the edges.
        """
        if BOX_EXTENT_M is None:
            return self._place(L.artwork_points(self.class_name))
        h = BOX_EXTENT_M / 2.0
        return self._place([(-h, -h), (h, -h), (h, h), (-h, h)])


def random_scene(rng, zone, mixed, scene_yaw):
    """Cards on the mat, plus the vehicle position that frames them.

    The real zones are columns:
        hazard  x 1.40, y 0.00 / -0.45          two cards, 0.45 m apart
        person  x 2.70, y -1.05 / -1.85 / -2.65 three cards, 0.80 m apart
    A frame is rendered around one zone, then the whole scene is rotated by
    scene_yaw about the zone centre so the cards land at an arbitrary angle in
    the image - which is what a heading error does to them in flight.

    Which two hazard signs stand on the two hazard cards is drawn uniformly
    from all nine.  The world file happens to ship Flammable and Explosive, but
    the task sheet decides which signs are out on the day, so a detector that
    has only ever seen those two would be blind to the other seven.
    """
    cards = []
    if zone == "hazard" or mixed:
        picks = rng.sample(L.hazard_names(), len(L.HAZARD_Y))
        for name, base_y in zip(picks, L.HAZARD_Y):
            cards.append(Card(name,
                              L.HAZARD_X + rng.uniform(-0.05, 0.05),
                              base_y + rng.uniform(-0.04, 0.04),
                              rng.uniform(-0.12, 0.12)))
    if zone == "person" or mixed:
        count = rng.randint(1, 3) if not mixed else rng.randint(2, 3)
        ys = list(L.PERSON_Y)
        rng.shuffle(ys)
        for name, base_y in zip([L.person_names()[0]] * count, ys[:count]):
            cards.append(Card(name,
                              L.PERSON_X + rng.uniform(-0.05, 0.05),
                              base_y + rng.uniform(-0.04, 0.04),
                              rng.uniform(-0.12, 0.12)))

    # Rotate the scene about its own centre so the cards are not always axis
    # aligned in the image.
    cx = sum(c.x for c in cards) / len(cards)
    cy = sum(c.y for c in cards) / len(cards)
    rc, rs = math.cos(scene_yaw), math.sin(scene_yaw)
    for card in cards:
        dx, dy = card.x - cx, card.y - cy
        card.x = cx + rc * dx - rs * dy
        card.y = cy + rs * dx + rc * dy
        card.yaw += scene_yaw

    # Camera: centred on the zone, offset by the hover error, at a plausible
    # altitude.  scene_yaw cancels out in `Camera` because the camera's yaw is
    # measured in the same world frame.
    cam_offset = rng.uniform(-0.22, 0.22), rng.uniform(-0.22, 0.22)
    alt = L.TASK_HEIGHT_M + rng.uniform(-0.12, 0.12) - L.CAM_MOUNT_DROP_M
    cam = Camera(
        position=(cx + cam_offset[0], cy + cam_offset[1], alt),
        yaw=rng.uniform(0.0, 2.0 * math.pi),
        pitch=math.radians(rng.uniform(-5.0, 5.0)),
        roll=math.radians(rng.uniform(-5.0, 5.0)),
        hfov=L.CAM_HFOV_RAD, width=L.CAM_WIDTH, height=L.CAM_HEIGHT)
    return cards, cam


# ---------------------------------------------------------------------------
# Background: the printed mat the cards lie on
# ---------------------------------------------------------------------------
def make_background(rng, cam):
    """Mat colour, printed grid, and the soft unevenness of a vinyl banner."""
    w, h = cam.width, cam.height
    base = np.array([238, 238, 236], np.float32)
    img = np.tile(base, (h, w, 1))

    # Large-scale illumination unevenness (a few percent), smooth enough that
    # it never looks like noise.
    ys, xs = np.mgrid[0:h, 0:w]
    for _ in range(2):
        fx_, fy_ = rng.uniform(0.6, 2.4), rng.uniform(0.6, 2.4)
        ph = rng.uniform(0, 2 * math.pi)
        img += (rng.uniform(-7, 7)
                * np.sin(xs / w * fx_ * 2 * math.pi + ph)
                * np.cos(ys / h * fy_ * 2 * math.pi + ph))[..., None]

    # Printed grid.  The real mat carries a printed grid; at 1.05 m altitude it
    # lands at a few px per cell, which is exactly the texture that confuses a
    # colour detector and should not confuse this one.
    cell_px = rng.uniform(22.0, 46.0)
    tint = np.array(rng.uniform(198, 224), np.float32)
    th = max(1, int(rng.uniform(0.8, 1.6)))
    for x in np.arange(rng.uniform(0, cell_px), w, cell_px):
        img[:, int(x):int(x) + th] = tint
    for y in np.arange(rng.uniform(0, cell_px), h, cell_px):
        img[int(y):int(y) + th, :] = tint

    noise = rng.uniform(1.5, 4.5)
    img += np.random.normal(0.0, noise, (h, w, 1))
    return Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), "RGB")


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def render(cards, cam, rng, background=None):
    """Draw the cards into a camera frame.  Returns (image, boxes).

    boxes are (class_name, xmin, ymin, xmax, ymax) in pixels, already clipped
    to the frame.  A card that lands entirely outside contributes nothing; a
    card that is half outside contributes a clipped box, which is deliberate -
    truncation at the frame edge is a real condition at 1.2 m and the model has
    to cope with it rather than being trained only on tidy centred cards.
    """
    if background is None:
        background = make_background(rng, cam)
    canvas = background.copy()

    boxes = []
    for card in cards:
        # Two quadrilaterals, both projected rather than one being scaled in
        # image space: the painted card (0.45 m square) and the annotation box
        # (the 0.336 x 0.291 m artwork).  They are not related by a single
        # factor, so an image-space scale would look right in the middle of the
        # frame and drift at the edges, where perspective is doing the most.
        paint, depth = cam.project(card.card_corners())
        # Skip a card whose corners are not all in front of the camera.
        if np.any(depth <= 1e-3):
            continue
        pts, _ = cam.project(card.box_corners())

        pxmin = max(0.0, float(paint[:, 0].min()))
        pymin = max(0.0, float(paint[:, 1].min()))
        pxmax = min(float(cam.width), float(paint[:, 0].max()))
        pymax = min(float(cam.height), float(paint[:, 1].max()))

        xmin = max(0.0, float(pts[:, 0].min()))
        ymin = max(0.0, float(pts[:, 1].min()))
        xmax = min(float(cam.width), float(pts[:, 0].max()))
        ymax = min(float(cam.height), float(pts[:, 1].max()))
        if xmax - xmin < 6.0 or ymax - ymin < 6.0:
            continue

        # Pixel size the card would have head-on, so the composed artwork keeps
        # the frame/face proportion no matter how it is tilted.
        span = max(paint[:, 0].max() - paint[:, 0].min(),
                   paint[:, 1].max() - paint[:, 1].min())
        card_px = int(max(16, min(1024, span * 1.35)))
        art = compose_card(card.class_name, card_px)
        side = card_px

        # Paste only over the card's footprint: the artwork is mapped from its
        # own square onto the projected quadrilateral with a homography, which
        # PIL wants in the src<-dst direction.
        pad = 2
        w = int(math.ceil(pxmax - pxmin)) + 2 * pad
        h = int(math.ceil(pymax - pymin)) + 2 * pad
        src = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
        dst = [(paint[i, 0] - pxmin + pad, paint[i, 1] - pymin + pad)
               for i in range(4)]
        h_local_to_img = homography(src, dst)
        try:
            h_img_to_local = np.linalg.inv(h_local_to_img)
        except np.linalg.LinAlgError:
            continue
        h_img_to_local /= h_img_to_local[2, 2]
        coeffs = tuple(h_img_to_local.flatten()[:8])
        warped = art.transform((w, h), Image.Transform.PERSPECTIVE, coeffs,
                               resample=Image.Resampling.BICUBIC)
        canvas.paste(warped, (int(pxmin) - pad, int(pymin) - pad), warped)

        boxes.append((card.class_name, xmin, ymin, xmax, ymax))

    return canvas, boxes


# ---------------------------------------------------------------------------
# Nuisance model
# ---------------------------------------------------------------------------
def augment(img, rng):
    """Degradations the flight link and the lighting actually introduce.

    Every one of these was chosen from something observed on this field, not
    from a generic augmentation list.  The JPEG step in particular is not
    decoration: the frames reach the ground station as CompressedImage and the
    ringing around a high-contrast frame edge is what a colour detector keys on
    and a learned one must learn to ignore.
    """
    if rng.random() < 0.85:
        img = ImageEnhance.Brightness(img).enhance(rng.uniform(0.68, 1.30))
    if rng.random() < 0.85:
        img = ImageEnhance.Contrast(img).enhance(rng.uniform(0.72, 1.28))
    if rng.random() < 0.5:
        img = ImageEnhance.Color(img).enhance(rng.uniform(0.70, 1.25))

    if rng.random() < 0.35:                      # white balance drift
        a = np.asarray(img, np.float32)
        a[..., 0] *= rng.uniform(0.92, 1.08)
        a[..., 2] *= rng.uniform(0.92, 1.08)
        img = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8), "RGB")

    if rng.random() < 0.45:                      # camera shake / motion
        img = img.filter(ImageFilter.GaussianBlur(rng.uniform(0.35, 1.15)))
    if rng.random() < 0.30:                      # slight defocus
        img = img.filter(ImageFilter.GaussianBlur(rng.uniform(1.0, 1.9)))

    if rng.random() < 0.55:                      # sensor noise
        a = np.asarray(img, np.float32)
        a += np.random.normal(0.0, rng.uniform(2.0, 7.5), a.shape)
        img = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8), "RGB")

    # A soft shadow band across the mat now and then.
    if rng.random() < 0.20:
        a = np.asarray(img, np.float32)
        w = img.width
        x0 = rng.uniform(-0.3, 0.9) * w
        width = rng.uniform(0.10, 0.35) * w
        ramp = np.clip(1.0 - np.abs(np.arange(w) - x0) / max(1.0, width), 0.0, 1.0)
        a *= (1.0 - 0.20 * ramp)[None, :, None]
        img = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8), "RGB")
    return img


def jpeg_roundtrip(img, rng):
    """Encode/decode at a link-like quality so the artefact is in the pixels."""
    import io
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=int(rng.uniform(38, 88)))
    buf.seek(0)
    return Image.open(buf).convert("RGB")


# ---------------------------------------------------------------------------
# Dataset writing
# ---------------------------------------------------------------------------
def voc_xml(filename, size, boxes):
    root = ET.Element("annotation")
    ET.SubElement(root, "folder").text = "images"
    ET.SubElement(root, "filename").text = filename
    ET.SubElement(root, "path").text = filename
    src = ET.SubElement(root, "source")
    ET.SubElement(src, "database").text = "gc_2026_synthetic"
    size_el = ET.SubElement(root, "size")
    ET.SubElement(size_el, "width").text = str(size[0])
    ET.SubElement(size_el, "height").text = str(size[1])
    ET.SubElement(size_el, "depth").text = "3"
    ET.SubElement(root, "segmented").text = "0"
    for name, xmin, ymin, xmax, ymax in boxes:
        obj = ET.SubElement(root, "object")
        ET.SubElement(obj, "name").text = name
        ET.SubElement(obj, "pose").text = "Unspecified"
        ET.SubElement(obj, "truncated").text = "0"
        ET.SubElement(obj, "difficult").text = "0"
        bb = ET.SubElement(obj, "bndbox")
        ET.SubElement(bb, "xmin").text = str(int(round(xmin)))
        ET.SubElement(bb, "ymin").text = str(int(round(ymin)))
        ET.SubElement(bb, "xmax").text = str(int(round(xmax)))
        ET.SubElement(bb, "ymax").text = str(int(round(ymax)))
    return ET.tostring(root, encoding="unicode")


def draw_preview(img, boxes):
    out = img.copy()
    d = ImageDraw.Draw(out)
    for name, xmin, ymin, xmax, ymax in boxes:
        color = (255, 0, 0) if L.zone_of(name) == "hazard" else (0, 128, 255)
        d.rectangle([xmin, ymin, xmax, ymax], outline=color, width=2)
        label = "%s" % name
        ty = max(0, int(ymin) - 12)
        d.rectangle([xmin, ty, xmin + 7 * len(label), ty + 12], fill=color)
        d.text((xmin + 2, ty + 1), label, fill=(255, 255, 255))
    return out


def make_sample(index, rng, mixed, zones=("hazard", "person")):
    # With --mixed both zones are rendered, so which one "zone" names is
    # irrelevant there; without it the two are drawn half and half.
    # zones 决定"哪几种场景被渲染"：("hazard",) 就是只画危险品、完全不画人员卡。
    zone = rng.choice(list(zones))
    scene_yaw = (rng.uniform(-math.radians(30), math.radians(30))
                 if rng.random() < 0.8 else rng.uniform(0.0, 2.0 * math.pi))
    cards, cam = random_scene(rng, zone, mixed, scene_yaw)
    img, boxes = render(cards, cam, rng)
    if not boxes:
        return None
    img = augment(img, rng)
    if rng.random() < 0.85:
        img = jpeg_roundtrip(img, rng)
    return img, boxes


def main():
    ap = argparse.ArgumentParser(
        description="Render a synthetic task-card dataset with exact labels.")
    ap.add_argument("--out", default=None, help="dataset directory")
    ap.add_argument("--count", type=int, default=4000,
                    help="number of frames to render (default 4000).  Roughly "
                         "half the frames carry the two hazard cards, so the "
                         "default yields about count boxes spread over the nine "
                         "hazard classes, and about count person boxes")
    ap.add_argument("--seed", type=int, default=20261006)
    ap.add_argument("--preview", default=None,
                    help="write annotated previews here and exit")
    ap.add_argument("--preview-count", type=int, default=8)
    ap.add_argument("--mixed", action="store_true",
                    help="put both zones in every frame (harder set)")
    ap.add_argument("--zones", default="both",
                    help="which zones to render: 'both' (default), 'hazard' "
                         "(the nine warning signs only - no person card at "
                         "all), or 'person'.  The person card is hand-drawn "
                         "artwork, so a synthetic portrait is a weaker stand-in "
                         "than a synthetic GB 2894 pictogram: when the real "
                         "photographs already cover the person class, render "
                         "'hazard' and let the photographs carry the person.")
    ap.add_argument("--box-extent", default="artwork",
                    help="what the annotation covers: 'artwork' (default, the "
                         "printed sign - matches the hand-labelled real set), "
                         "'card' (the whole %.3f m card including its frame "
                         "bars), or an explicit square size in metres"
                         % L.CARD_FRAME_M)
    args = ap.parse_args()

    global BOX_EXTENT_M
    if args.box_extent == "artwork":
        BOX_EXTENT_M = None
    elif args.box_extent == "card":
        BOX_EXTENT_M = L.CARD_FRAME_M
    else:
        try:
            BOX_EXTENT_M = float(args.box_extent)
        except ValueError:
            ap.error("--box-extent wants 'artwork', 'card', or a size in metres")
        if not 0.02 <= BOX_EXTENT_M <= 1.0:
            ap.error("--box-extent %.3f m is outside 0.02..1.0 m" % BOX_EXTENT_M)

    if not args.preview and not args.out:
        ap.error("give --out to build a dataset, or --preview to look first")

    # zones 解析。'both' + --mixed = 每帧都同时有危险品和人员卡；
    # --mixed 只在 both 下说得通（单 zone + 混合 = 自相矛盾）。
    zones = {"both": ("hazard", "person"),
             "hazard": ("hazard",),
             "person": ("person",)}.get(args.zones)
    if zones is None:
        ap.error("--zones wants 'both', 'hazard' or 'person'")
    if args.mixed and len(zones) == 1:
        ap.error("--mixed needs --zones both (mixing one zone is meaningless)")
    print("zones     : %s%s" % (", ".join(zones),
                                " (mixed)" if args.mixed else ""))
    if "person" not in zones:
        print("            person class will have NO synthetic samples; "
              "the real photographs must carry it")

    if args.preview:
        os.makedirs(args.preview, exist_ok=True)
        for i in range(args.preview_count):
            rng = random.Random(args.seed + i)
            sample = make_sample(i, rng, args.mixed, zones)
            if sample is None:
                continue
            img, boxes = sample
            img.save(os.path.join(args.preview, "preview_%03d.jpg" % i),
                     quality=92)
            draw_preview(img, boxes).save(
                os.path.join(args.preview, "preview_%03d_boxes.png" % i))
            print("preview_%03d  %d box(es): %s"
                  % (i, len(boxes), ", ".join(b[0] for b in boxes)))
        print("\nwrote %d preview pair(s) to %s" % (args.preview_count, args.preview))
        return 0

    img_dir = os.path.join(args.out, "images")
    ann_dir = os.path.join(args.out, "annotations")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(ann_dir, exist_ok=True)

    rng = random.Random(args.seed)
    written = 0
    per_class = {name: 0 for name in L.CLASS_NAMES}
    attempts = 0
    while written < args.count:
        attempts += 1
        if attempts > args.count * 12:
            print("stopping: too many empty frames", file=sys.stderr)
            break
        sample = make_sample(written, rng, args.mixed, zones)
        if sample is None:
            continue
        img, boxes = sample
        stem = "synth_%06d" % written
        img.save(os.path.join(img_dir, stem + ".jpg"),
                 quality=int(rng.uniform(60, 95)))
        with open(os.path.join(ann_dir, stem + ".xml"), "w",
                  encoding="utf-8") as handle:
            handle.write(voc_xml(stem + ".jpg", img.size, boxes))
        for name, _, _, _, _ in boxes:
            per_class[name] += 1
        written += 1
        if written % 200 == 0:
            print("  %d/%d" % (written, args.count))

    with open(os.path.join(args.out, "label_map.pbtxt"), "w",
              encoding="utf-8") as handle:
        handle.write(L.label_map_pbtxt())

    print("\nwrote %d frames to %s" % (written, args.out))
    print("boxes per class:")
    for name in L.CLASS_NAMES:
        print("  %-26s %6d" % (name, per_class[name]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
