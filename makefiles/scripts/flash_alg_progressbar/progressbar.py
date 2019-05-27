# -*- coding: UTF-8 -*-
# Date    : 2018/08/07
# Author  : Snow Yang
# Mail    : yangsw@mxchip.com

import sys
from time import sleep, time
from terminal import get_terminal_size


class ProgressBar():
    def __init__(self, max_value, marker='█', fill='░'):
        self.marker = marker
        self.max_value = max_value
        self.fill = fill
        w = get_terminal_size()[0]
        self.width = w - 5 - 9 - 12 - 9
        self.last_width = 0
        self.last_value = 0
        self.last_time = 0
        self.start_time = 0

    def update(self, value):
        if value > self.max_value:
            raise ValueError('value to large')
        marker_width = value * self.width / self.max_value
        if marker_width <= self.last_width:
            return
        speed = 0
        t = time()
        if self.last_time > 0:
            speed = (value - self.last_value) / (t - self.last_time)
        else:
            self.start_time = t
        self.last_value = value
        self.last_time = t
        self.last_width = marker_width
        fill_width = self.width - marker_width
        sys.stdout.write('\r' + self.marker * marker_width +
                         self.fill * fill_width + ' %3d' % (value * 100 / self.max_value) + '%' +
                         ' %4d KiB' % (value / 1024) +
                         ' %5.1f KiB/s' % (speed / 1024) +
                         ' %4d s' % (t - self.start_time))
        sys.stdout.flush()

if __name__ == "__main__":
    bar = ProgressBar(1024 * 36)
    for i in range(36):
        bar.update((i + 1) * 1024)
        sleep(0.1)
