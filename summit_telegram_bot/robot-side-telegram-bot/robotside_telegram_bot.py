#!/usr/bin/env python
# -*- coding: utf-8 -*-

import requests
import logging

# Enable logging
logging.basicConfig(format='%(asctime)s-%(name)s-%(levelname)s-%(message)s',
                    level=logging.INFO)
logger = logging.getLogger(__name__)


def upload_to_server(filename):
    f = open(filename)
    r = requests.post(url='http://ingmec.ual.es/robot-summit/upload.php',
                      data={'title': 'file'},  files={'UPLOAD': f})
    if r.status_code != 200:
        logger.error('Error uploading file to server!')


def main():
    upload_to_server('/tmp/last_camera_img.png')


if __name__ == '__main__':
    main()
