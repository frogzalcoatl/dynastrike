// Consider switching to this if I can't figure out how to make a dynamic quadtree.

import { Entity } from "../core/entity";
import { Scene } from "../core/scene";
import { Collision } from "../physics/collide";
import { Box } from "../types";

export class HashGrid {
	private static stride: number = 67108863;

	public cells: Map<number, Entity[]> = new Map();
	public scene: Scene;
	public cellSize: number;
	constructor(scene: Scene, cellSize: number) {
		this.scene = scene;
		this.cellSize = cellSize;
	}

	public insert(box: Box, entity: Entity): void {
		const endX: number = box.maxX >> this.cellSize;
		const endY: number = box.maxY >> this.cellSize;
		for (let x: number = box.minX >> this.cellSize; x <= endX; x++) {
			for (let y: number = box.minY >> this.cellSize; y <= endY; y++) {
				const key = x + y * HashGrid.stride;
				if (!this.cells.get(key)) {
					this.cells.set(key, [entity]);
					continue;
				}
				this.cells.get(key)!.push(entity);
			}
		}
	}

	public query(box: Box): Set<Entity> {
		const entities: Set<Entity> = new Set<Entity>();
		const endX: number = box.maxX >> this.cellSize;
		const endY: number = box.maxY >> this.cellSize;
		for (let x: number = box.minX >> this.cellSize; x <= endX; x++) {
			for (let y: number = box.minY >> this.cellSize; y <= endY; y++) {
				const key = x + y * HashGrid.stride;
				const cell = this.cells.get(key);
				if (cell === undefined) {
					continue;
				}
				for (let i = 0; i < cell.length; i++) {
					entities.add(cell[i]);
				}
			}
		}
		return entities;
	}

	public collisions() {
		const checked: Set<number> = new Set<number>();
		for (const cell of this.cells.values()) {
			const length: number = cell.length;
			if (length < 2) {
				continue;
			}
			for (let i: number = 0; i < length; i++) {
				const instance: Entity = cell[i];
				for (let j: number = i + 1; j < length; j++) {
					const other: Entity = cell[j];
					if (instance.isStatic && other.isStatic) {
						continue;
					}
					const pairIndex: number = this.getPairIndex(instance.index, other.index);
					if (checked.has(pairIndex)) {
						continue;
					}
					checked.add(pairIndex);
					Collision.collide(instance, other);
					this.earthlyShackles(other);
				}
				this.earthlyShackles(instance);
			}
		}
	}

	public clear() {
		this.cells.clear();
	};

	private getPairIndex(instanceIndex: number, otherIndex: number): number {
		return instanceIndex < otherIndex ? otherIndex * otherIndex + instanceIndex : instanceIndex * instanceIndex + otherIndex;
	}

	private earthlyShackles(entity: Entity): void {
		if (entity.positionX < this.scene.box.minX) {
			entity.positionX = this.scene.box.minX;
			entity.velocityX *= -entity.restitution;
		} else if (entity.positionX > this.scene.box.maxX) {
			entity.positionX = this.scene.box.maxX;
			entity.velocityX *= -entity.restitution;
		}
		if (entity.positionY < this.scene.box.minY) {
			entity.positionY = this.scene.box.minY;
			entity.velocityY *= -entity.restitution;
		} else if (entity.positionY > this.scene.box.maxY) {
			entity.positionY = this.scene.box.maxY;
			entity.velocityY *= -entity.restitution;
		}
	}
};