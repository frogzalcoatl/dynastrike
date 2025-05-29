export class Entity {
	public x: number;
	public y: number;
	constructor(x: number, y: number) {
		this.x = x;
		this.y = y;
	}

	public tick(deltaTime: number): void {

	}
}

export class Scene {
	public entities: Set<Entity>;
	public lastTickTime: number;
	public intervalIndex: number;
	constructor(tickrate: number) {
		this.entities = new Set();
		if (tickrate === 0) {
			this.lastTickTime = -1;
			this.intervalIndex = -1;
		} else {
			this.lastTickTime = performance.now();
			this.intervalIndex = setInterval(() => this.tick(), tickrate);
		}
	}

	public addEntity(entity: Entity) {
		if (this.entities.has(entity)) return false;
		this.entities.add(entity);
		return true;
	}

	public removeEntity(entity: Entity) {
		return this.entities.delete(entity);
	}

	public destroy() {
		if (this.lastTickTime === -1) {
			clearInterval(this.intervalIndex);
		}
	}

	public tick(): void {
		let deltaTime = 1;
		if (this.lastTickTime === -1) {
			const currentTime = performance.now();
			deltaTime = Math.min((currentTime - this.lastTickTime) / 1000, 0.1);
			this.lastTickTime = currentTime;
		}
		for (const entity of this.entities.values()) {
			entity.tick(deltaTime);
		}
	}
}