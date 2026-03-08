# Memory

Lazy, chainable query system for persistent robot data. Stores timestamped observations in SQLite with vector similarity (sqlite-vec), full-text (FTS5), spatial (R*Tree), and temporal indexes.

```sh no-result
rm -f /tmp/memory_readme.db
```

## Quick start

```python session=memory ansi=false no-result
from dimos.memory.impl.sqlite import SqliteStore

store = SqliteStore("/tmp/memory_readme.db")
session = store.session()
```

Open a store, get a session, create a stream:

```python session=memory ansi=false
logs = session.stream("logs", str)
print(logs)
```

<!--Result:-->
```
Stream[str]("logs")
```

Append observations and query them:

```python session=memory ansi=false
logs.append("Motor started", ts=1.0, tags={"level": "info"})
logs.append("Joint 3 fault", ts=2.0, tags={"level": "error"})
logs.append("Motor stopped", ts=3.0, tags={"level": "info"})

print(logs.summary())
```

<!--Result:-->
```
Stream[str]("logs"): 3 items, 1970-01-01 00:00:01 — 1970-01-01 00:00:03 (2.0s)
```

## Observations

Each observation wraps a payload with metadata:

```python session=memory ansi=false
obs = logs.first()
print(obs)
print(f"id={obs.id}, ts={obs.ts}, tags={obs.tags}")
```

<!--Result:-->
```
Observation(id=1, ts=1.0, pose=None, tags={'level': 'info'})
id=1, ts=1.0, tags={'level': 'info'}
```

- `id` — auto-assigned integer
- `ts` — timestamp (float, seconds)
- `pose` — optional 3D position + orientation
- `tags` — key-value metadata dict
- `parent_id` — lineage tracking (set by transforms)
- `data` — the payload (lazily loaded from DB)

### Lazy payload loading

Metadata is always in memory. The `.data` property triggers a single-row `SELECT` on first access, then caches:

```python skip
obs = logs.first()
obs.ts          # already in memory
obs.tags        # already in memory
obs.data        # NOW loads the blob and decodes it
obs.data        # cached — second access is free
```

Payloads are decoded by the thread that fetched them. To pass observations across threads, call `.load()` first:

```python skip
obs = logs.first().load()  # force-loads .data, safe to pass to another thread
```

## Streams are lazy queries

Filter methods return new `Stream` instances — nothing executes until a terminal is called:

```python session=memory ansi=false
query = logs.after(1.0).filter_tags(level="info").order_by("ts", desc=True).limit(5)
print(query)
# nothing has hit the database yet
```

<!--Result:-->
```
Stream[str]("logs") | after(t=1.0) | tags(level='info') | order(ts, desc) | limit(5)
```

Each call clones the stream with updated query parameters. The underlying `StreamQuery` compiles to SQL only at terminal time.

### Filters

| Method | Description |
|--------|-------------|
| `.after(t)` | `ts > t` |
| `.before(t)` | `ts < t` |
| `.time_range(t1, t2)` | `t1 <= ts <= t2` |
| `.at(t, tolerance=1.0)` | `\|ts - t\| <= tolerance` |
| `.near(pose, radius)` | R*Tree bounding box + exact distance post-filter |
| `.filter_tags(**kv)` | JSON tag field matching |

### Ordering and pagination

| Method | Description |
|--------|-------------|
| `.order_by(field, desc=False)` | Sort by `"ts"` or `"id"` |
| `.limit(k)` | Cap results |
| `.offset(n)` | Skip first n results |

## Terminals execute the query

| Terminal | Returns | Description |
|----------|---------|-------------|
| `.fetch()` | `ObservationSet` | All matching rows (lazy payloads) |
| `.fetch_pages(batch_size=128)` | `Iterator[list[Observation]]` | Paginated iteration |
| `.count()` | `int` | `SELECT COUNT(*)`, no payload loading |
| `.first()` | `Observation` | First by current ordering; raises `LookupError` if empty |
| `.last()` | `Observation` | Most recent by `ts` |
| `.exists()` | `bool` | `count() > 0` |
| `.summary()` | `str` | Count, time range, duration |
| `.get_time_range()` | `(float, float)` | `(first.ts, last.ts)` |

List-like access also works:

```python session=memory ansi=false
print(f"len={len(logs)}, bool={bool(logs)}")
print(f"logs[0] = {logs[0]}")
print(f"logs[-1] = {logs[-1]}")
```

<!--Result:-->
```
len=3, bool=True
logs[0] = Observation(id=1, ts=1.0, pose=None, tags={'level': 'info'})
logs[-1] = Observation(id=3, ts=3.0, pose=None, tags={'level': 'info'})
```

Iteration uses paginated fetching under the hood:

```python session=memory ansi=false
for obs in logs.after(1.5):
    print(obs)
```

<!--Result:-->
```
Observation(id=2, ts=2.0, pose=None, tags={'level': 'error'})
Observation(id=3, ts=3.0, pose=None, tags={'level': 'info'})
```

## ObservationSet

`.fetch()` returns an `ObservationSet` — an in-memory result set that is itself a `Stream`. All filters and terminals work on it, re-evaluating in memory without hitting the database:

```python session=memory ansi=false
results = logs.fetch()
print(results)
print(f"len={len(results)}")

# re-filter in memory — no DB hit
errors = results.filter_tags(level="error").fetch()
print(errors)
print(errors[0])
```

<!--Result:-->
```
ObservationSet[str](3 items)
len=3
ObservationSet[str](1 items)
Observation(id=2, ts=2.0, pose=None, tags={'level': 'error'})
```

ObservationSet is read-only — `.append()` raises `TypeError`.

When you chain filters on an ObservationSet, it downgrades to a plain Stream backed by the in-memory list, so it doesn't carry the full result set through the chain.

## Transforms

`.transform()` applies a function to each observation's payload. Without `.store()`, it runs entirely in memory:

```python session=memory ansi=false
upper = logs.transform(lambda s: s.upper())
print(upper)
print(upper.fetch())
for obs in upper.fetch():
    print(obs.data)
```

<!--Result:-->
```
TransformStream[?](Stream[str]("logs") -> PerItemTransformer)
ObservationSet[?](3 items)
MOTOR STARTED
JOINT 3 FAULT
MOTOR STOPPED
```

Return `None` to skip an item, return a `list` to fan-out:

```python skip
# Filter: skip short messages
long = logs.transform(lambda s: s if len(s) > 10 else None)

# Fan-out: split into words
words = logs.transform(lambda s: s.split())
```

### Storing transforms

`.store(name)` materializes a transform into a new named stream in the database:

```python skip
# Default: backfill existing + subscribe to new appends
embeddings = images.transform(EmbeddingTransformer(clip)).store("clip_embeddings")

# Live only: skip backfill, only process new appends
embeddings = images.transform(EmbeddingTransformer(clip), live=True).store("clip_embeddings")

# Backfill only: process existing data, don't subscribe
embeddings = images.transform(EmbeddingTransformer(clip), backfill_only=True).store("clip_embeddings")
```

| Mode | Processes existing data | Subscribes to new appends |
|------|-------------------------|---------------------------|
| default | yes | yes |
| `live=True` | no | yes |
| `backfill_only=True` | yes | no |

The output stream kind is auto-detected from the transformer: `EmbeddingTransformer` and `TextEmbeddingTransformer` create an `EmbeddingStream` with vec0 index, `CaptionTransformer` creates a `TextStream` with FTS index.

Storing also records **parent lineage** — which source stream produced the derived stream. This powers `project_to`.

### Built-in transformers

| Transformer | Input | Output | Stored as |
|---|---|---|---|
| `PerItemTransformer(fn)` | any | any | `Stream` |
| `QualityWindowTransformer(quality_fn, window)` | any | same type | `Stream` — keeps best-quality item per time window |
| `CaptionTransformer(model)` | Image | `str` | `TextStream` with FTS index |
| `EmbeddingTransformer(model)` | Image/any | `Embedding` | `EmbeddingStream` with vec0 index |
| `TextEmbeddingTransformer(model)` | `str` | `Embedding` | `EmbeddingStream` with vec0 index |

`QualityWindowTransformer` buffers observations within a time window and emits only the one with the highest quality score. Useful for sharpness filtering on camera frames:

```python skip
from dimos.memory.transformer import QualityWindowTransformer

sharp = images.transform(
    QualityWindowTransformer(quality_fn=lambda img: img.sharpness, window=0.5)
).store("sharp_frames")
```

## Specialized streams

### TextStream — full-text search

```python session=memory ansi=false
text = session.text_stream("events")
text.append("Motor fault on joint 3", ts=1.0)
text.append("Battery low warning", ts=2.0)
text.append("Motor recovered", ts=3.0)

results = text.search_text("motor").fetch()
print(results)
for obs in results:
    print(f"  {obs.data}")
```

<!--Result:-->
```
ObservationSet[str](2 items)
  Motor recovered
  Motor fault on joint 3
```

Uses SQLite FTS5. Results are ranked by relevance. Optional `k` parameter limits results.

### EmbeddingStream — vector similarity search

```python skip
embs = session.embedding_stream("clip_embs", vec_dimensions=512)
embs.append(embedding_vector, ts=1.0)

results = embs.search_embedding([0.5, 0.3, ...], k=5).fetch()
```

Uses sqlite-vec (vec0) for cosine similarity. `search_embedding` accepts:
- Pre-computed `Embedding` or `list[float]`
- A `str` — auto-embedded via the stream's model (`embed_text`)
- An `Image` or other object — auto-embedded via the stream's model (`embed`)

Results are `EmbeddingObservation` with `.similarity` (0–1 cosine) and `.embedding` (convenience alias for `.data`).

## Lineage and project_to

When you store a transform, each derived observation tracks its `parent_id`. Use `.project_to()` to follow the lineage chain back to a source stream:

```python skip
images = session.stream("images", Image)
embeddings = images.transform(EmbeddingTransformer(clip)).store("clip_embeddings")

# Search returns EmbeddingObservation — .data is the Embedding, not the source Image
results = embeddings.search_embedding("a hallway", k=5).fetch()
results[0].similarity  # cosine similarity (0–1)
results[0].embedding   # the Embedding vector
results[0].data        # also the Embedding (same as .embedding)

# To get source images, project back through the lineage chain
image_results = embeddings.search_embedding("office", k=5).project_to(images).fetch()

# Multi-hop works too: embeddings → sharp_frames → raw_images
image_results = embeddings.search_embedding("office", k=5).project_to(raw_images).fetch()
```

## Reactive subscriptions

Streams emit observations as they're appended:

```python skip
images.subscribe(lambda obs: print(f"new frame at {obs.ts}"))

# Filters work on subscriptions too:
images.after(10.0).filter_tags(cam="front").subscribe(handle_front_cam)

# Or get the raw RxPY Observable:
observable = images.observable()
```

Under the hood this is an RxPY Observable on the backend's `Subject`. Embedding and lineage filters are skipped for live filtering (they need DB context); temporal, spatial, and tag filters work.

## Codecs

Payloads are BLOB-encoded via auto-selected codecs:

| Codec | Used for | Strategy |
|-------|----------|----------|
| `JpegCodec` | `Image` | TurboJPEG lossy compression (~10-20x smaller), preserves `frame_id` |
| `LcmCodec` | `DimosMsg` types | LCM binary encoding (lossless) |
| `PickleCodec` | everything else | Python pickle (fallback) |

`codec_for_type(payload_type)` auto-selects the best codec. This is transparent — you never need to specify a codec manually.

## Session management

### Listing streams

```python session=memory ansi=false
for info in session.list_streams():
    print(f"{info.name}: {info.stream_kind}, {info.count} items")
```

<!--Result:-->
```
logs: stream, 3 items
events: text, 3 items
```

### Context managers

```python skip
with SqliteStore("memory.db") as store:
    session = store.session()
    # ... use session ...
# store.stop() called automatically
```

### Pose provider

Auto-attach pose to every appended observation:

```python skip
images = session.stream("images", Image, pose_provider=robot.get_pose)
images.append(frame)  # pose is auto-filled from pose_provider()
```

### Persistence

Data persists across restarts. Reopen the same database and streams pick up where they left off:

```python skip
store = SqliteStore("memory.db")
session = store.session()
images = session.stream("images", Image)
results = images.after(100.0).fetch()  # picks up old data
```

The `_streams` meta-table tracks stream names, payload types (as module paths), stream kind, parent lineage, and embedding dimensions.

## MemoryModule — blueprint integration

In a robot blueprint, `MemoryModule` wires input streams to memory:

```python skip
class MyMemory(MemoryModule):
    camera: In[Image]

    def start(self):
        super().start()

        # Record camera input to a named stream (name/type inferred from input)
        self.image_memory = self.memory(self.camera)

        # With quality filtering at 2 fps (keeps sharpest frame per window)
        self.image_memory = self.memory(self.camera, fps=2)

        # Build derived streams
        self.embeddings = self.image_memory.transform(
            EmbeddingTransformer(CLIPModel()), live=True
        ).store("clip_embeddings")
```

## Utilities

### Bulk import

```python skip
from dimos.memory.ingest import ingest

# Import from any iterable of (timestamp, payload) tuples
count = ingest(images, replay.iterate_ts())

# With pose interpolation from an odometry source
count = ingest(images, replay.iterate_ts(), pose_source=odom_replay)
```

### Rerun export

```python skip
from dimos.memory.rerun import to_rerun

# Log a stream's observations to Rerun timeline
count = to_rerun(images)
count = to_rerun(images, entity_path="memory/camera")
```

```python session=memory no-result
store.stop()
```

```sh no-result
rm -f /tmp/memory_readme.db
```
