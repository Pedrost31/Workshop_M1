-- Schema d'historique des capteurs du robot Yanshee (TimescaleDB)

CREATE EXTENSION IF NOT EXISTS timescaledb;

CREATE TABLE IF NOT EXISTS readings (
    time         TIMESTAMPTZ      NOT NULL DEFAULT now(),
    robot_up     BOOLEAN          NOT NULL,
    perime       BOOLEAN,
    dist_g       DOUBLE PRECISION,   -- ultrason gauche (cm)
    dist_c       DOUBLE PRECISION,   -- ultrason centre (cm)
    dist_d       DOUBLE PRECISION,   -- ultrason droite (cm)
    gaz          DOUBLE PRECISION,   -- MQ-2 (0-1023)
    vapeur       DOUBLE PRECISION,   -- steam / eau (0-1023)
    lum          DOUBLE PRECISION,   -- LDR (0-1023)
    battery_pct  DOUBLE PRECISION,   -- batterie (%)
    charging     BOOLEAN,            -- en charge
    mv_etat      TEXT,               -- etat mouvement (marche, obstacle, ...)
    latency_ms   DOUBLE PRECISION    -- latence requete /capteurs (ms)
);

-- Transforme la table en hypertable orientee temps
SELECT create_hypertable('readings', 'time', if_not_exists => TRUE);

CREATE INDEX IF NOT EXISTS readings_time_idx ON readings (time DESC);

-- Politique de retention : on garde 90 jours de mesures brutes
SELECT add_retention_policy('readings', INTERVAL '90 days', if_not_exists => TRUE);
