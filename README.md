# JPEG DECODER

a simple jpeg decoder, implement from zero to learn its compress algorithm.

参考了一些网络上的JPEG文章，并未参考其实现，在实现上均按自己的理解进行，开发中，所参考的文章将在后续贴上链接

## 进度

1. 区段解析
2. SOF0解析
3. DQT解析：量化表头解析完成，值尚未进行
4. DHT解析：完成，验证中
5. SOS解析：完成
6. 图像解析，MCU、block...，未开始

IDCT:

$$ result[i][j] = \frac{1}{4}\sum{_{x=0} ^7}\sum{_{y=0} ^7}C_x C_y cos(\frac{(2i + 1)x\pi}{16}) cos (\frac{(2j + 1)y\pi}{16}) block[x][y] 
$$
$$
C_0 = \frac{1}{\sqrt{2}}​
$$
$$
C_i = 1, \forall i > 0
$$
