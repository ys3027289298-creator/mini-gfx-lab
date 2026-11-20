CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -pthread

all: ringbuf

ringbuf: src/ringbuf.c
	$(CC) $(CFLAGS) $< -o $@

# ThreadSanitizer / AddressSanitizer+UBsan builds for the concurrent paths
tsan: src/ringbuf.c
	$(CC) -O1 -g -std=c11 -Wall -Wextra -pthread -fsanitize=thread $< -o ringbuf_tsan

asan: src/ringbuf.c
	$(CC) -O1 -g -std=c11 -Wall -Wextra -pthread -fsanitize=address,undefined $< -o ringbuf_asan

test: ringbuf
	./ringbuf

# WSL2/某些内核的默认 ASLR 布局与 TSan 冲突 (unexpected memory mapping /
# 启动时段错误); setarch -R 关掉 ASLR 即可, 没有 setarch 的系统回退直跑
test-sanitizers: tsan asan
	(setarch -R ./ringbuf_tsan 2>/dev/null || ./ringbuf_tsan)
	./ringbuf_asan

clean:
	rm -f ringbuf ringbuf_tsan ringbuf_asan
