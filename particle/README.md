# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度。

时间（帧 dt）与随机（散射角/速度/寿命抖动）全部经
[`simenv`](../simenv/README.md) 注入，本模块不直接调用任何时间或随机 API，
固定时钟 + 固定种子即可逐粒子复现。

```bash
make test
```
