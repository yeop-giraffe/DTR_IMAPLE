import os
import math
import sys
import cv2
import time
import torch
import argparse
import numpy as np
from pathlib import Path
from collections import Counter
import torch.backends.cudnn as cudnn
from utils.general import set_logging
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size,
                           check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args,
                           scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

# --------------------------- Serial Port ------------------------------
import serial

# Open the serial port
# ser = serial.Serial('/dev/ttyS0', 9600)  # Replace 'COM1' with the actual port and 9600 with the baud rate
# time.sleep(2)


# Classes
class_id = {'0': 'Circle',
            '1': 'Triangle',
            '2': 'Square',
            '3': 'Purple Balloon',
            '4': 'Green Balloon',
            '5': 'Red Balloon'
            }


def send_serial_command(vert, hor, stop, forward, backward, cls):
    # Convert the signal values to bytes (assuming they are integers)
    # signal_bytes = f"{vert},{hor},{stop},{forward},{backward} \n".encode('utf-8')
    # ser.write(signal_bytes)
    print(f"Sending (class {class_id[str(int(cls))]}): Y:{vert}, X:{hor}, S:{stop}, F:{forward}, B:{backward} ")
    # cmd_text = f"Y:{vert}, X:{hor}, S:{stop}, F:{forward}, B:{backward}\n"
    #cmd_text = f"Y:8, X:-4, S:0, F:0, B:1\n"
    # ser.write(cmd_text.encode())
    #time.sleep(1)
    # return (vert, hor, stop, forward, backward)


def send_control_commands(det, gn, target_rect, image_shape):
    c0, r0, tr_w, tr_h = target_rect
    aImg = image_shape[0] * image_shape[1]

    max_area = -999999999
    det_idx = -1
    for i, (*xyxy, conf, cls) in enumerate(det):
        # Look only at the classes: square, triangle, circle

        if cls.item() > 2:
            continue

        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh

        # Pick the target with the largest bounding box area
        d_to_center = (xywh[0] - 0.5)**2 + (xywh[1] - 0.5)**2
        current_area = np.cos(np.pi / 2 * d_to_center) * (xywh[2] * xywh[3])
        if current_area > max_area:
            max_area = current_area
            det_idx = i

    if det_idx == -1:
        return -1, (0, 0, 0, 0, 0) # No target detected, sending nothing

    # Get the info of the selected target
    *xyxy, conf, cls = det[det_idx]

    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
    xywh = [v * 255 for v in xywh]

    x, y, w, h = bbox_rel(*xyxy)  # Calculate bbox center coordinates
    vertical = int(y - c0)  # Calculate vertical displacement
    horizontal = int(x - r0)  # Calculate horizontal displacement
    l = math.sqrt(vertical ** 2 + horizontal ** 2)
    theta = math.atan2(vertical, horizontal)  # in radians
    theta = math.degrees(theta)  # in degrees
    aBB = w * h
    # Check if the center of the bounding box is within the red box
    centered = (r0 - tr_h <= x <= r0 + tr_h) and (c0 - tr_w <= y <= c0 + tr_w)

    # normalized offset commands
    horizontal_norm = int(2 * horizontal / image_shape[1] * 100)
    vertical_norm = int(2 * vertical / image_shape[0] * 100)

    if centered:
        if is_close(aImg, aBB):
            cmd = 0, 0, 1, 0, 0
        elif is_close(aImg, aBB, thresh=0.3):
            cmd = 0, 0, 0, 0, 1
        else:
            cmd = 0, 0, 0, 1, 0
    else:
        #print(f"Length: {l}, Theta: {theta}")
        cmd = int(vertical_norm), int(horizontal_norm), 0, 0, 0

    send_serial_command(*cmd, cls)

    return det_idx, cmd

    #             |   far  | close(0.7) | too close
    # ----------------------------------------------
    # centered    |   (1)  |    (2)     |    (3)
    # ----------------------------------------------
    # not centered|     signal=(y, x, 0, 0, 0)

    # (1) signal = (0, 0, 0, 1, 0)
    # (2) signal = (0, 0, 1, 0, 0)
    # (3) signal = (0, 0, 0, 0, 1)


def display_command(img, command):

    # Create a bigger image with space in the bottom for text displaying
    gui_image = np.zeros((int(1.2 * img.shape[0]), img.shape[1], img.shape[2]), dtype=img.dtype)
    gui_image[:img.shape[0], ...] = img

    # Draw the command text in the new image
    action = 'Stop' if command[2] == 1 else ('Forward' if command[3] == 1 else ('Backward' if command[4] == 1 else 'None'))

    cv2.putText(
        gui_image, f'Y: {command[0]}', (int(0.05 * gui_image.shape[1]), int(0.95 * gui_image.shape[0])),
        cv2.FONT_HERSHEY_SIMPLEX, 1.0, [255, 255, 255], 1
    )

    cv2.putText(
        gui_image, f'X: {command[1]}', (int(0.3 * gui_image.shape[1]), int(0.95 * gui_image.shape[0])),
        cv2.FONT_HERSHEY_SIMPLEX, 1.0, [255, 255, 255], 1
    )

    cv2.putText(
        gui_image, f'Action: {action}', (int(0.55 * gui_image.shape[1]), int(0.95 * gui_image.shape[0])),
        cv2.FONT_HERSHEY_SIMPLEX, 1.0, [255, 255, 255], 1
    )

    return gui_image

# -------------------------- Object Tracking ---------------------------
import skimage
from sort import *

# ------------- OpenCV object tracker implementations ------------------
OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.legacy.TrackerCSRT_create,
    "kcf": cv2.legacy.TrackerKCF_create,
    "boosting": cv2.legacy.TrackerBoosting_create,
    "mil": cv2.legacy.TrackerMIL_create,
    "tld": cv2.legacy.TrackerTLD_create,
    "medianflow": cv2.legacy.TrackerMedianFlow_create,
    "mosse": cv2.legacy.TrackerMOSSE_create
}
tracker_name = 'kcf'

# ----------------------------- Functions ------------------------------
'''Computer Color for every box and track'''
palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)

def compute_color_for_labels(label):
    color = [int(int(p * (label ** 2 - label + 1)) % 255) for p in palette]
    return tuple(color)


"""" Calculates the relative bounding box from absolute pixel values. """
def bbox_rel(*xyxy):
    bbox_left = min([xyxy[0].item(), xyxy[2].item()])
    bbox_top = min([xyxy[1].item(), xyxy[3].item()])
    bbox_w = abs(xyxy[0].item() - xyxy[2].item())
    bbox_h = abs(xyxy[1].item() - xyxy[3].item())
    x_c = (bbox_left + bbox_w / 2)
    y_c = (bbox_top + bbox_h / 2)
    w = bbox_w
    h = bbox_h
    return x_c, y_c, w, h


"""Function to Draw Bounding boxes"""
def draw_boxes(img, bbox, identities=None, categories=None,
               names=None, color_box=None, offset=(0, 0), target_idx=-1):

    for i, box in enumerate(bbox):
        x1, y1, x2, y2 = [int(i) for i in box]
        x1 += offset[0]
        x2 += offset[0]
        y1 += offset[1]
        y2 += offset[1]
        cat = int(categories[i]) if categories is not None else 0
        n = names.get(str(cat), 'Unknown')
        id = int(identities[i]) if identities is not None else 0
        data = (int((box[0] + box[2]) / 2), (int((box[1] + box[3]) / 2)))
        label = str(n)

        if color_box:
            color = compute_color_for_labels(id)
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), (255, 191, 0), -1)
            cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 255], 1)
            cv2.circle(img, data, 3, color, -1)
        else:

            if i == target_idx:
                color = (0, 255, 0) # Color is green to show the target we are pursuing
            else:
                color = (255, 191, 0)

            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), color, -1)
            cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 255], 1)
            cv2.circle(img, data, 3, color, -1)

    return img


"""Funtion to compare area of bounding box to area of Image"""
def is_close(aImg, aBB, thresh=1/9):
    if aBB/ aImg >= thresh: # threshold can be calibrated
        return 1
    return 0


# ..............................................................................


@torch.no_grad()
def detect(weights=ROOT / 'yolov5n.pt',
           source=ROOT / 'yolov5/data/images',
           data=ROOT / 'yolov5/data/coco128.yaml',
           imgsz=(640, 640), conf_thres=0.25, iou_thres=0.45,
           max_det=1000, device='cpu', view_img=False,
           save_txt=False, save_conf=False, save_crop=False,
           nosave=False, classes=None, agnostic_nms=False,
           augment=False, visualize=False, update=False,
           project=ROOT / 'runs/detect', name='exp',
           exist_ok=False, line_thickness=2, hide_labels=False,
           hide_conf=False, half=False, dnn=False, display_labels=False,
           blur_obj=False, color_box=False, ):
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)

    if is_url and is_file:
        source = check_file(source)  # download
    
    # Area of Img
    aImg = imgsz[0]*imgsz[1]
    aIm = (aImg - 320**2 ) // 100 # standardized
    
    # .... Initialize SORT ....
    sort_tracker = Sort(max_age=5, min_hits=2, iou_threshold=0.2)
    track_color_id = 0
    # .........................

    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)

    set_logging()
    device = select_device(device)
    half &= device.type != 'cpu'

    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)

    half &= (pt or jit or onnx or engine) and device.type != 'cpu'
    if pt or jit:
        model.model.half() if half else model.model.float()

    if webcam:
        cudnn.benchmark = True
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
        bs = len(dataset)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
        bs = 1
    vid_path, vid_writer = [None] * bs, [None] * bs

    t0 = time.time()

    dt, seen = [0.0, 0.0, 0.0], 0

    n_frames = 100
    pred = []
    multi_tracker = None
    # last_detection_time = time.time()
    # n = 1
    
    for frame_idx, (path, im, im0s, vid_cap, s) in enumerate(dataset):

        # Command to dispaly
        cmd = (1, -2, 0, 1, 0)

        # Center of Img ... absolute
        r0 = im0s[0].shape[1] // 2
        c0 = im0s[0].shape[0] // 2
        h0 = 40
        w0 = 40
        
        image_to_show = im0s[0]
        current_time = time.time()

        # Area of Img
        #aImg = im0s[0].shape[0] * im0s[0].shape[0][1]
        #aIm = (aImg - 320 ** 2) // 100  # standardized
        
        # We use YOLO5 if 'n' frames has passed
        # *** Modify to run YOLO every t seconds, rather than n frames ***
        if frame_idx % n_frames == 0:
        # We use YOLO5 every n seconds
        # if last_detection_time - current_time >= n:
            t1 = time_sync()
            im = torch.from_numpy(im).to(device)
            im = im.half() if half else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            t2 = time_sync()
            dt[0] += t2 - t1
        
            # Inference
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize)
            t3 = time_sync()
            dt[1] += t3 - t2

            # NMS
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
            dt[2] += time_sync() - t3

            for i, det in enumerate(pred):  # per image
                seen += 1
                if webcam:  # batch_size >= 1
                    p, im0, frame = path[i], im0s[i].copy(), dataset.count
                    s += f'{i}: '
                else:
                    p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

                # print(f'{det.shape[0]} objects detected')
                p = Path(p)
                save_path = str(save_dir / p.name)
                txt_path = str(save_dir / 'labels' / p.stem) + (
                    '' if dataset.mode == 'image' else f'_{frame}')  # im.txt
                s += '%gx%g ' % im.shape[2:]
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
                imc = im0.copy() if save_crop else im0  # for save_crop
                if len(det):
                    det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                    # Print results
                    for c in det[:, 5].unique():
                        n = (det[:, 5] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # Write results and serial communication
                    target_det_idx, cmd = send_control_commands(det, gn, (c0, r0, w0, h0), im0s[i].shape)
                     

                    # ..................USE TRACK FUNCTION....................
                    # pass an empty array to sort
                    dets_to_sort = np.empty((0, 6))

                    # NOTE: We send in detected object class too
                    for x1, y1, x2, y2, conf, detclass in det.cpu().detach().numpy():
                        dets_to_sort = np.vstack((dets_to_sort,
                                                  np.array([x1, y1, x2, y2,
                                                            conf, detclass])))
                    
                    # Run SORT
                    tracked_dets = sort_tracker.update(dets_to_sort)
                    tracks = sort_tracker.getTrackers()

                image_to_show = im0
                if save_img:
                    if dataset.mode == 'image':
                        cv2.imwrite(save_path, im0)
                    else:
                        if vid_path != save_path:
                            vid_path = save_path
                            if isinstance(vid_writer, cv2.VideoWriter):
                                vid_writer.release()
                            if vid_cap:
                                fps = vid_cap.get(cv2.CAP_PROP_FPS)
                                w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            else:
                                fps, w, h = 30, im0.shape[1], im0.shape[0]
                                save_path += '.mp4'
                            vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                        vid_writer.write(im0)

            last_detection_time = current_time
            multi_tracker = None
        else:
            # If we do not run YOLO initialize OpenCV's multi-object tracker
            if multi_tracker is None:
                multi_tracker = cv2.legacy.MultiTracker_create()
                
                boxes = [(int(det[0]), int(det[1]), int(det[2] - det[0]), int(det[3] - det[1])) for det in pred[0]]
                confs = [det[4] for det in pred[0]]
                ids = [det[5] for det in pred[0]]

                for box in boxes:
                    # dets_to_sort = np.vstack((dets_to_sort, np.array([x1, y1, x2, y2, conf, detclass])))
                    tracker = OPENCV_OBJECT_TRACKERS[tracker_name]()

                    # Initialize the tracker with the selected box
                    tracker.init(image_to_show, box)

                    # Add the tracker to the multi-tracker
                    multi_tracker.add(tracker, image_to_show, box)

            # grab the updated bounding box coordinates (if any) for each
            # object that is being tracked
            success, boxes = multi_tracker.update(image_to_show)
            
            # Run SORT
            dets_to_sort = np.array(
                [[b[0], b[1], b[0] + b[2], b[1] + b[3], c, cl] for b, c, cl in zip(boxes, confs, ids)])

            if len(dets_to_sort) > 0:
                tracked_dets = sort_tracker.update(dets_to_sort)
                tracks = sort_tracker.getTrackers()

                # for box in boxes:
                #    (x, y, w, h) = [int(v) for v in box]
                #    cv2.rectangle(image_to_show, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # ............ Remove comments to enable tracks ............
                # for track in tracks:
                    # if color_box:
                        # color = compute_color_for_labels(track_color_id)
                        # [cv2.line(image_to_show, (int(track.centroidarr[i][0]), int(track.centroidarr[i][1])),
                                  # (int(track.centroidarr[i + 1][0]), int(track.centroidarr[i + 1][1])),
                                  # color, thickness=3) for i, _ in enumerate(track.centroidarr)
                         # if i < len(track.centroidarr) - 1]
                        # track_color_id = track_color_id + 1
                    # else:
                        # [cv2.line(image_to_show, (int(track.centroidarr[i][0]), int(track.centroidarr[i][1])),
                                  # (int(track.centroidarr[i + 1][0]), int(track.centroidarr[i + 1][1])),
                                  # (124, 252, 0), thickness=3) for i, _ in enumerate(track.centroidarr)
                         # if i < len(track.centroidarr) - 1]

                # draw boxes for visualization
                if len(tracked_dets) > 0:
                    bbox_xyxy = tracked_dets[:, :4]
                    identities = tracked_dets[:, 8]
                    categories = tracked_dets[:, 4]
                    draw_boxes(image_to_show, bbox_xyxy, identities, categories, class_id, color_box, target_idx=target_det_idx)

                # Write results and serial communication
                target_det_idx, cmd = send_control_commands(
                    torch.tensor(tracked_dets[:, [0, 1, 2, 3, 5, 4]], dtype=torch.float32),
                    gn, (c0, r0, w0, h0), image_to_show.shape
                )
            else:
                target_det_idx, cmd = send_control_commands([], gn, (c0, r0, w0, h0), image_to_show.shape)

            # draw box at the center of the image
            cv2.rectangle(image_to_show, (r0 - h0, c0 + w0), (r0 + h0, c0 - w0), (0, 0, 255), 2)
            # cv2.circle(im0s[0], (r0,c0), radius=1, color=(0, 0, 255), thickness=-3)

        if view_img:
            gui_image = display_command(image_to_show, cmd)
            cv2.imshow(str(Path('0')), gui_image)
            cv2.waitKey(1)

    if update:
        strip_optimizer(weights)

    if vid_cap:
        vid_cap.release()


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--blur-obj', action='store_true', help='Blur Detected Objects')
    parser.add_argument('--color-box', action='store_true', help='Change color of every box and track')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt


def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    detect(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
