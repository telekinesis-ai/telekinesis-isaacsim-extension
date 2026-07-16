"""Client that sends a 6D zero joint position vector to the benchmark server, repeatedly, over
either a REST API or a WebSocket, and reports round-trip latency as well as the one-way send
(client send -> server receive) and receive (server send -> client receive) latencies.

Run with:
    python api_websocket_benchmark_client.py --mode api --requests 200
    python api_websocket_benchmark_client.py --mode websocket --requests 200
"""

import argparse
import asyncio
import json
import statistics
import time

import httpx
import websockets

JOINT_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def print_latency_stats(name: str, latencies_ms: list[float]) -> None:
    print(f"{name}: mean={statistics.mean(latencies_ms):.3f} ms, "
          f"median={statistics.median(latencies_ms):.3f} ms, "
          f"stdev={statistics.stdev(latencies_ms):.3f} ms, "
          f"min={min(latencies_ms):.3f} ms, max={max(latencies_ms):.3f} ms")


def print_statistics(
    mode: str,
    round_trip_times_ms: list[float],
    send_latencies_ms: list[float],
    receive_latencies_ms: list[float],
    server_processing_times_ms: list[float],
) -> None:
    print(f"\n--- {mode} results ({len(round_trip_times_ms)} requests) ---")
    print_latency_stats("round trip", round_trip_times_ms)
    print_latency_stats("send (client send -> server receive)", send_latencies_ms)
    print_latency_stats("receive (server send -> client receive)", receive_latencies_ms)
    print(f"server processing: mean={statistics.mean(server_processing_times_ms):.3f} ms, "
          f"median={statistics.median(server_processing_times_ms):.3f} ms")


async def run_api(server_url: str, num_requests: int) -> None:
    round_trip_times_ms = []
    send_latencies_ms = []
    receive_latencies_ms = []
    server_processing_times_ms = []
    async with httpx.AsyncClient() as client:
        for _ in range(num_requests):
            client_send_perf = time.perf_counter()
            client_send_time = time.time()
            response = await client.post(
                f"{server_url}/joint_positions",
                json={"positions": JOINT_POSITIONS, "client_send_time": client_send_time},
            )
            client_receive_perf = time.perf_counter()
            client_receive_time = time.time()
            data = response.json()
            round_trip_times_ms.append((client_receive_perf - client_send_perf) * 1000)
            send_latencies_ms.append((data["server_receive_time"] - client_send_time) * 1000)
            receive_latencies_ms.append((client_receive_time - data["server_send_time"]) * 1000)
            server_processing_times_ms.append((data["server_send_time"] - data["server_receive_time"]) * 1000)
    print_statistics("api", round_trip_times_ms, send_latencies_ms, receive_latencies_ms, server_processing_times_ms)


async def run_websocket(server_url: str, num_requests: int) -> None:
    round_trip_times_ms = []
    send_latencies_ms = []
    receive_latencies_ms = []
    server_processing_times_ms = []
    async with websockets.connect(f"{server_url.replace('http', 'ws')}/ws/joint_positions") as websocket:
        for _ in range(num_requests):
            client_send_perf = time.perf_counter()
            client_send_time = time.time()
            await websocket.send(
                json.dumps({"positions": JOINT_POSITIONS, "client_send_time": client_send_time})
            )
            response = await websocket.recv()
            client_receive_perf = time.perf_counter()
            client_receive_time = time.time()
            data = json.loads(response)
            round_trip_times_ms.append((client_receive_perf - client_send_perf) * 1000)
            send_latencies_ms.append((data["server_receive_time"] - client_send_time) * 1000)
            receive_latencies_ms.append((client_receive_time - data["server_send_time"]) * 1000)
            server_processing_times_ms.append((data["server_send_time"] - data["server_receive_time"]) * 1000)
    print_statistics(
        "websocket", round_trip_times_ms, send_latencies_ms, receive_latencies_ms, server_processing_times_ms
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["api", "websocket"], required=True, help="Transport to benchmark.")
    parser.add_argument("--server-url", default="http://127.0.0.1:8000", help="Base URL of the server.")
    parser.add_argument("--requests", type=int, default=200, help="Number of requests to send.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.mode == "api":
        asyncio.run(run_api(args.server_url, args.requests))
    else:
        asyncio.run(run_websocket(args.server_url, args.requests))


if __name__ == "__main__":
    main()
