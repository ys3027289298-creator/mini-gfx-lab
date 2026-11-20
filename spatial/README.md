# mini-spatial-audio

双耳定位：方位角→ITD/IID、反距离衰减、多声源软限幅混音。

```bash
make test
```

## 完整渲染链

`src/render_chain.c` 把直达声扩展为四段串联：头相关卷积（HRTF）、房间混响、
多普勒、距离衰减。段顺序与每段增益都在 `ChainConfig` 里配置，`make chain-test`
跑自检并打印开销对比。

```bash
make chain-test
```

自检覆盖：

- 每段对闭式解：ITD 抽头位置、IID 左右增益、1/d 距离律、多普勒比值；
- 串联输出与**逐段手工计算**的参考实现逐样本比较（参考实现独立重写，不复用
  链里的段函数），容差 `1e-4`，实测 `1e-8`；
- 单块渲染与 61/97/128 帧分块渲染逐样本一致，块缝处不新增阶跃；
- 源速度逐块翻转时输出无爆音；
- 四段同时开启 vs 各段单独开启的耗时（`us/block` 与实时倍率）。

```
config           us/block   realtime
hrtf                3.99    1338.1x
reverb              2.23    2388.3x
doppler             1.77    3020.4x
distance            0.25   21080.4x
all four            7.97     669.2x
sum of single       8.24     647.4x
```

四段同时开启约等于各段单独开启之和（比值 0.97），说明串联框架本身几乎不引入
额外开销。
