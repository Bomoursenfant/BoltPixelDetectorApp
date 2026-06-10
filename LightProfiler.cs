using System.Diagnostics;
using System.Collections.Concurrent;
using System.IO;

namespace BoltPixelDetectorApp;

internal static class LightProfiler
{
    private static readonly ConcurrentDictionary<string, long> _acc = new();
    private static readonly ConcurrentDictionary<string, long> _count = new();
    private static readonly bool _enabled;

    static LightProfiler()
    {
        try
        {
            var outPath = Environment.GetEnvironmentVariable("LIGHT_PROFILER_OUT");
            if (!string.IsNullOrEmpty(outPath))
            {
                _enabled = true;
                AppDomain.CurrentDomain.ProcessExit += (_, __) =>
                {
                    try
                    {
                        using var sw = new StreamWriter(outPath, append: false);
                        sw.WriteLine("LightProfiler Snapshot");
                        foreach (var kv in Snapshot())
                        {
                            sw.WriteLine($"{kv.Key}\t{kv.Value.totalMs}ms\tcount={kv.Value.count}");
                        }
                    }
                    catch
                    {
                        // swallow - best effort snapshot
                    }
                };
            }
        }
        catch
        {
            // ignore
        }
    }

    public static IDisposable Measure(string key)
    {
        if (!_enabled)
            return NoopScope.Instance;

        return new Scope(key);
    }

    public static IReadOnlyDictionary<string, (long totalMs, long count)> Snapshot()
    {
        var dict = new Dictionary<string, (long, long)>();
        foreach (var kv in _acc)
        {
            _count.TryGetValue(kv.Key, out var c);
            // convert stopwatch ticks to milliseconds
            long ms = (long)(kv.Value * 1000.0 / Stopwatch.Frequency);
            dict[kv.Key] = (ms, c);
        }

        return dict;
    }

    private sealed class NoopScope : IDisposable
    {
        public static readonly NoopScope Instance = new();
        public void Dispose()
        {
        }
    }

    private sealed class Scope : IDisposable
    {
        private readonly string _key;
        private readonly long _start;
        public Scope(string key)
        {
            _key = key;
            _start = Stopwatch.GetTimestamp();
        }

        public void Dispose()
        {
            long end = Stopwatch.GetTimestamp();
            long elapsed = end - _start;
            _acc.AddOrUpdate(_key, elapsed, (_, old) => old + elapsed);
            _count.AddOrUpdate(_key, 1, (_, old) => old + 1);
        }
    }
}
