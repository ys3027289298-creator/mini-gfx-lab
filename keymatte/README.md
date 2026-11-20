# keymatte

精灵带底色导出、与其他图合成后边缘出现一圈彩色描边。该模块按色差抠除底色并做边缘处理：

- 色度空间距离（忽略亮度）作为抠除依据，阈值按底色与墨色的色差自动推导，不需手调魔数；
- 每像素覆盖率 alpha 由色差线性映射，底色像素为 0、纯墨色像素为 255、抗锯齿边缘为中间值；
- 去污染（decontamination）按覆盖率反解出本色，边缘不再残留底色色相；
- 边缘收缩（可分离最小值滤波）与羽化（滑动窗口均值，只会降低 alpha、不会凭空造出覆盖），半透明像素按 `alpha × 源 alpha` 保留；
- 全部定点整数运算，每个阶段对像素做单趟扫描，耗时与像素数线性。

`make test` 跑自检，包括与手工抠图（按解析覆盖率生成的参考蒙版）逐像素比对：alpha 误差 ≤ 1，合成结果误差 ≤ 3，彩边像素数为 0，以及两种分辨率下的每像素耗时对比。

```c
Keyer ky;
int tol = tol_for_colors(key_rgb, ink_rgb);   /* 或直接给 tol_milli */
keyer_init(&ky, w, h, key_rgb, tol);

Image src = { w, h, pixels };                 /* RGBA，源 alpha 可为半透明 */
Matte out, scratch;
matte_extract(&ky, &src, &out, &scratch, /*shrink=*/1, /*feather=*/2);
/* out.px: straight colour + coverage，可直接合成或写入 PNG 的 RGBA */
```
