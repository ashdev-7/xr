"""Peak RSS (kB) and wall time of a command, measured like an evaluator would (resource.getrusage on the child)."""
import resource, subprocess, sys, time
t0 = time.perf_counter(); subprocess.run(sys.argv[1:], check=True); wall = time.perf_counter() - t0
ru = resource.getrusage(resource.RUSAGE_CHILDREN)
print(f"peak_rss={ru.ru_maxrss/1024:.2f} MB  wall={wall*1000:.0f} ms  cpu={(ru.ru_utime+ru.ru_stime)*1000:.0f} ms")
