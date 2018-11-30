import numpy as np
import cv2
import os
import time

class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
               'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
               'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
               'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
               'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
               'kite', 'baseball bat', 'baseball glove', 'skateboard',
               'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
               'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
               'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
               'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
               'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
               'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
               'teddy bear', 'hair drier', 'toothbrush']

data_path = os.path.abspath('/home/erdou/Documents/code/cpp/RSCAR/data/data_tmp/')
video_path = '/home/erdou/Documents/code/cpp/RSCAR/data/in_video.mp4'
element_type = ['scores', 'masks', 'class_ids', 'rois']
interest_name = ['person', 'car', 'bus', 'truck', 'motorcycle', 'bicycle']
dictionary = {interest_name[0]: 40,
              interest_name[1]: 70,
              interest_name[2]: 100,
              interest_name[3]: 130,
              interest_name[4]: 160,
              interest_name[5]: 290,
              }


def get_min_rect(mask, roi):
    mask_cv = mask.astype(np.uint8) * 255
    im2, contours, hierarchy = cv2.findContours(mask_cv, 1, 2)
    max_area = 0
    if len(contours) > 1:
        for item in contours:
            area = cv2.contourArea(item)
            if area:
                max_area = area
                cnt = item
    else:
        cnt = contours[0]
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect) + [roi[1], roi[0]]
    box = np.int0(box)
    return box


def get_mask(data, frame):
    N = len(data[element_type[0]])
    name = data[element_type[2]]
    mask_bool = data_detect[element_type[1]]
    mask = np.zeros(mask_bool[:, :, 0].shape, dtype=np.uint8)
    roi = data[element_type[3]]
    for i in range(N):
        if class_names[name[i]] in interest_name:
            mask[roi[i][0]:roi[i][2], roi[i][1]:roi[i][3]][mask_bool[roi[i][0]:roi[i][2], roi[i][1]:roi[i][3], i]] = \
            dictionary[class_names[name[i]]]
            # cv2.rectangle(mask, (roi[i][1], roi[i][0]), (roi[i][3], roi[i][2]), (255, 255, 255), 2)
            cv2.rectangle(frame, (roi[i][1], roi[i][0]), (roi[i][3], roi[i][2]), (255, 0, 0), 2)
            # cv2.putText(frame, class_names[name[i]], ((roi[i][1] + roi[i][3])/2, (roi[i][0] + roi[i][2])/2), cv2.FONT_ITALIC, 1, (0, 0, 128), 2)
            # box = get_min_rect(mask_bool[roi[i][0]:roi[i][2], roi[i][1]:roi[i][3], i], roi[i])
            # cv2.drawContours(mask, [box], 0, (255, 255, 255), 2)
            # cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
    return roi, mask


if __name__ == '__main__':
    video_capture = cv2.VideoCapture(video_path)
    video_writer = cv2.VideoWriter('/home/erdou/Documents/code/cpp/RSCAR/data/mask_video.mp4',
                                   cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 30, (1080, 1080))
    f = open('/home/erdou/Documents/code/cpp/RSCAR/data/mask_bbox.txt', 'w')
    for i in range(15999):
        data_detect = np.load(os.path.join(data_path, str(i) + '.npz'))
        # tmp_matrix = data_detect[element_type[1]][:, :, 0].astype(np.uint8) * 255
        success, frame = video_capture.read()
        frame = cv2.resize(frame, (1024, 1024))
        roi, mask = get_mask(data_detect, frame)
        f.write(str(len(roi)) + '\n')
        for item in roi:
            item_tmp = item * 1080 / 1024.
            item_tmp = item_tmp.astype(np.int32)
            string_tmp = str(item_tmp[1]) + ' ' + str(item_tmp[0]) + ' ' + str(item_tmp[3]) + ' ' + str(
                item_tmp[2]) + '\n'
            f.write(string_tmp)
        cv2.imshow('mask', mask)
        cv2.imshow('rcnn', frame)
        key = cv2.waitKey(1)
        frame[:, :, 1] = mask
        frame = cv2.resize(frame, (1080, 1080))

        video_writer.write(frame)
        if key == 113:
            break
    f.close()
    video_writer.release()
