FROM python:3.15-slim

ENV PYTHONUNBUFFERED=1

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY collector.py .
RUN mkdir -p /app/static

EXPOSE 8000

CMD ["python", "collector.py"]
