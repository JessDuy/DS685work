CREATE EXTENSION IF NOT EXISTS vector;

CREATE TABLE IF NOT EXISTS detections (
  id BIGSERIAL PRIMARY KEY,
  ts TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  frame_id TEXT,
  class TEXT,
  score REAL,
  x_min INT, y_min INT, x_max INT, y_max INT,
  embedding VECTOR(2048),
  source TEXT,
  note TEXT
);

CREATE INDEX IF NOT EXISTS idx_detections_embedding
ON detections USING ivfflat (embedding vector_l2_ops) WITH (lists = 100);
