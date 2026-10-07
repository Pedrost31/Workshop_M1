"""
Notifier Yanshee (multi-canal)
==============================

Recoit les webhooks d'Alertmanager et notifie par EMAIL et/ou SMS.

Canaux :
  - EMAIL : via un serveur SMTP (Gmail, Brevo, OVH, Outlook, Mailtrap...)
  - SMS   : via l'API Free Mobile (uniquement vers sa propre ligne Free)

Configuration (variables d'environnement) :
  NOTIFY_CHANNELS   : "email", "sms" ou "email,sms" (defaut : auto selon ce qui est configure)
  NOTIFY_DRY_RUN    : "true" -> n'envoie rien, logge seulement

  # --- Email (SMTP) ---
  SMTP_HOST, SMTP_PORT (587), SMTP_USER, SMTP_PASS
  SMTP_FROM           : expediteur (defaut = SMTP_USER)
  SMTP_TO             : destinataires separes par des virgules
  SMTP_STARTTLS       : "true" (defaut) pour le port 587
  SMTP_SSL            : "true" pour le port 465 (SSL direct)

  # --- SMS (Free Mobile) ---
  FREE_MOBILE_USER / FREE_MOBILE_PASS  ou  FREE_MOBILE_ACCOUNTS (JSON)
"""

import json
import logging
import os
import smtplib
from email.message import EmailMessage

import requests
from flask import Flask, jsonify, request
from waitress import serve

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("notifier")


def _bool(name, default=False):
    val = os.environ.get(name)
    if val is None:
        return default
    return val.strip().lower() in ("1", "true", "yes", "oui")


# Compat : NOTIFY_DRY_RUN prioritaire, sinon ancien SMS_DRY_RUN
DRY_RUN = _bool("NOTIFY_DRY_RUN", _bool("SMS_DRY_RUN", False))

# --- Config email ------------------------------------------------------------
# Transport : "smtp" (classique) ou "brevo" (API HTTP sur 443, contourne les
# reseaux qui bloquent le SMTP sortant).
EMAIL_TRANSPORT = os.environ.get("EMAIL_TRANSPORT", "smtp").strip().lower()

SMTP_HOST = os.environ.get("SMTP_HOST", "").strip()
SMTP_PORT = int(os.environ.get("SMTP_PORT", "587"))
SMTP_USER = os.environ.get("SMTP_USER", "").strip()
SMTP_PASS = os.environ.get("SMTP_PASS", "")
SMTP_FROM = os.environ.get("SMTP_FROM", SMTP_USER).strip()
SMTP_TO = [a.strip() for a in os.environ.get("SMTP_TO", "").split(",") if a.strip()]
SMTP_STARTTLS = _bool("SMTP_STARTTLS", True)
SMTP_SSL = _bool("SMTP_SSL", False)

# API Brevo (https://api.brevo.com) - cle dans "SMTP & API > API Keys"
BREVO_API_KEY = os.environ.get("BREVO_API_KEY", "").strip()
BREVO_URL = "https://api.brevo.com/v3/smtp/email"

if EMAIL_TRANSPORT == "brevo":
    EMAIL_ENABLED = bool(BREVO_API_KEY and SMTP_FROM and SMTP_TO)
else:
    EMAIL_ENABLED = bool(SMTP_HOST and SMTP_TO)

# --- Config SMS (Free Mobile) ------------------------------------------------
FREE_SMS_URL = "https://smsapi.free-mobile.fr/sendmsg"


def load_sms_accounts():
    raw = os.environ.get("FREE_MOBILE_ACCOUNTS", "").strip()
    if raw:
        try:
            accounts = json.loads(raw)
            if isinstance(accounts, list) and accounts:
                return accounts
            log.error("FREE_MOBILE_ACCOUNTS doit etre une liste JSON non vide")
        except json.JSONDecodeError as e:
            log.error("FREE_MOBILE_ACCOUNTS invalide: %s", e)
    user = os.environ.get("FREE_MOBILE_USER", "").strip()
    pwd = os.environ.get("FREE_MOBILE_PASS", "").strip()
    if user and pwd:
        return [{"user": user, "pass": pwd, "nom": "principal"}]
    return []


SMS_ACCOUNTS = load_sms_accounts()
SMS_ENABLED = bool(SMS_ACCOUNTS)

# --- Choix des canaux actifs -------------------------------------------------
_requested = [c.strip().lower() for c in os.environ.get("NOTIFY_CHANNELS", "").split(",") if c.strip()]
if _requested:
    CHANNELS = _requested
else:
    CHANNELS = []
    if EMAIL_ENABLED:
        CHANNELS.append("email")
    if SMS_ENABLED:
        CHANNELS.append("sms")

if not CHANNELS:
    log.warning(
        "Aucun canal de notification configure (ni SMTP_* pour l'email, ni FREE_MOBILE_* pour le SMS)."
    )
else:
    log.info("Canaux actifs: %s (dry_run=%s)", ", ".join(CHANNELS), DRY_RUN)


# --- Envoi email -------------------------------------------------------------
def send_email(subject, body):
    if "email" not in CHANNELS:
        return {"canal": "email", "ignore": "non actif"}
    if not EMAIL_ENABLED:
        return {"canal": "email", "ok": False, "erreur": "email non configure"}
    if DRY_RUN:
        log.info("[DRY_RUN] EMAIL (%s) a %s\nSujet: %s\n%s",
                 EMAIL_TRANSPORT, SMTP_TO, subject, body)
        return {"canal": "email", "transport": EMAIL_TRANSPORT, "ok": True, "dry_run": True}

    if EMAIL_TRANSPORT == "brevo":
        return _send_email_brevo(subject, body)
    return _send_email_smtp(subject, body)


def _send_email_brevo(subject, body):
    try:
        r = requests.post(
            BREVO_URL,
            headers={
                "api-key": BREVO_API_KEY,
                "accept": "application/json",
                "content-type": "application/json",
            },
            json={
                "sender": {"email": SMTP_FROM},
                "to": [{"email": a} for a in SMTP_TO],
                "subject": subject,
                "textContent": body,
            },
            timeout=15,
        )
        ok = r.status_code in (200, 201, 202)
        if not ok:
            log.error("Brevo HTTP %s: %s", r.status_code, r.text[:300])
        return {"canal": "email", "transport": "brevo", "ok": ok,
                "status": r.status_code, "destinataires": SMTP_TO}
    except Exception as e:  # noqa: BLE001
        log.error("Envoi Brevo impossible: %s", e)
        return {"canal": "email", "transport": "brevo", "ok": False, "erreur": str(e)}


def _send_email_smtp(subject, body):
    msg = EmailMessage()
    msg["Subject"] = subject
    msg["From"] = SMTP_FROM
    msg["To"] = ", ".join(SMTP_TO)
    msg.set_content(body)
    try:
        if SMTP_SSL:
            server = smtplib.SMTP_SSL(SMTP_HOST, SMTP_PORT, timeout=20)
        else:
            server = smtplib.SMTP(SMTP_HOST, SMTP_PORT, timeout=20)
            if SMTP_STARTTLS:
                server.starttls()
        if SMTP_USER:
            server.login(SMTP_USER, SMTP_PASS)
        server.send_message(msg)
        server.quit()
        return {"canal": "email", "ok": True, "destinataires": SMTP_TO}
    except Exception as e:  # noqa: BLE001
        log.error("Envoi email impossible: %s", e)
        return {"canal": "email", "ok": False, "erreur": str(e)}


# --- Envoi SMS ---------------------------------------------------------------
def send_sms(text):
    if "sms" not in CHANNELS:
        return [{"canal": "sms", "ignore": "non actif"}]
    text = text[:960]
    if DRY_RUN:
        log.info("[DRY_RUN] SMS:\n%s", text)
        return [{"canal": "sms", "ok": True, "dry_run": True}]
    results = []
    for acc in SMS_ACCOUNTS:
        nom = acc.get("nom", acc.get("user", "?"))
        try:
            r = requests.get(
                FREE_SMS_URL,
                params={"user": acc["user"], "pass": acc["pass"], "msg": text},
                timeout=10,
            )
            ok = r.status_code == 200
            results.append({"canal": "sms", "nom": nom, "ok": ok, "status": r.status_code})
        except Exception as e:  # noqa: BLE001
            log.error("SMS vers %s impossible: %s", nom, e)
            results.append({"canal": "sms", "nom": nom, "ok": False, "erreur": str(e)})
    return results


# --- Mise en forme des alertes ----------------------------------------------
PREFIX = {"critical": "[CRITIQUE]", "warning": "[ALERTE]"}


def format_alert(alert):
    status = alert.get("status", "firing")
    labels = alert.get("labels", {})
    ann = alert.get("annotations", {})
    severity = labels.get("severity", "warning")
    tete = "[RESOLU]" if status == "resolved" else PREFIX.get(severity, "[ALERTE]")
    summary = ann.get("summary", labels.get("alertname", "Alerte Yanshee"))
    desc = ann.get("description", "")
    line = f"{tete} {summary}"
    if desc:
        line += f"\n    {desc}"
    return line, status, severity


def build_subject(alerts_meta):
    n = len(alerts_meta)
    if any(sev == "critical" and st == "firing" for _, st, sev in alerts_meta):
        return f"[Yanshee][CRITIQUE] {n} alerte(s)"
    if all(st == "resolved" for _, st, _ in alerts_meta):
        return f"[Yanshee][RESOLU] {n} alerte(s) terminee(s)"
    return f"[Yanshee] {n} alerte(s)"


app = Flask(__name__)


@app.route("/alert", methods=["POST"])
def alert():
    payload = request.get_json(force=True, silent=True) or {}
    alerts = payload.get("alerts", [])
    if not alerts:
        return jsonify(sent=0, detail="aucune alerte dans le payload")

    meta = [format_alert(a) for a in alerts]
    lines = [m[0] for m in meta]
    body = "Monitoring Yanshee\n\n" + "\n\n".join(lines)
    subject = build_subject(meta)
    log.info("%d alerte(s) recue(s) d'Alertmanager -> %s", len(alerts), ", ".join(CHANNELS) or "aucun canal")

    results = {"email": send_email(subject, body), "sms": send_sms(body)}
    return jsonify(sent=len(alerts), canaux=CHANNELS, resultats=results)


@app.route("/test")
def test():
    subject = "[Yanshee][TEST] Notifier operationnel"
    body = "Monitoring Yanshee\n\n[TEST] Si tu lis ceci, les notifications fonctionnent."
    results = {"email": send_email(subject, body), "sms": send_sms(body)}
    return jsonify(canaux=CHANNELS, resultats=results)


@app.route("/healthz")
def healthz():
    return jsonify(
        status="ok",
        canaux=CHANNELS,
        email_configure=EMAIL_ENABLED,
        email_transport=EMAIL_TRANSPORT,
        sms_configure=SMS_ENABLED,
        dry_run=DRY_RUN,
    )


if __name__ == "__main__":
    serve(app, host="0.0.0.0", port=5001, threads=4)
