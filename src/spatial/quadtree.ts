import { Entity } from "../core/entity";

export class QuadTree {
	public entities: Entity[] = [];
	public childTopLeft: QuadTree | null = null;
	public childTopRight: QuadTree | null = null;
	public childBottomLeft: QuadTree | null = null;
	public childBottomRight: QuadTree | null = null;
	public hasChildren: boolean = false;
	public minX: number;
	public minY: number;
	public maxX: number;
	public maxY: number;
	public level: number;
	constructor(minX: number, minY: number, maxX: number, maxY: number, level: number) {
		this.minX = minX;
		this.minY = minY;
		this.maxX = maxX;
		this.maxY = maxY;
		this.level = level;
	}

	public split(): void {
		const width: number = (this.minX + this.maxX) / 2;
		const height: number = (this.minY + this.maxY) / 2;
		const level: number = this.level - 1;
		this.childTopLeft = new QuadTree(this.minX, this.minY, width, height, level);
		this.childTopRight = new QuadTree(width, this.minY, this.maxX, height, level);
		this.childBottomLeft = new QuadTree(this.minX, height, width, this.maxY, level);
		this.childBottomRight = new QuadTree(width, height, this.maxX, this.maxY, level);
		this.hasChildren = true;
		const entitiesCount: number = this.entities.length;
		for (let i = 0; i < entitiesCount; i++) {
			this.insert(this.entities[i]);
		}
	}

	public insert(entity: Entity): void {
		const minX: number = entity.minX;
		const minY: number = entity.minY;
		const maxX: number = entity.maxX;
		const maxY: number = entity.maxY;
		if (this.hasChildren) {
			const tl: QuadTree = this.childTopLeft!;
			const tr: QuadTree = this.childTopRight!;
			const bl: QuadTree = this.childBottomLeft!;
			const br: QuadTree = this.childBottomRight!;
			if (maxX > tl.minX && minX < tl.maxX && maxY > tl.minY && minY < tl.maxY) {
				tl.insert(entity);
			}
			if (maxX > tr.minX && minX < tr.maxX && maxY > tr.minY && minY < tr.maxY) {
				tr.insert(entity);
			}
			if (maxX > bl.minX && minX < bl.maxX && maxY > bl.minY && minY < bl.maxY) {
				bl.insert(entity);
			}
			if (maxX > br.minX && minX < br.maxX && maxY > br.minY && minY < br.maxY) {
				br.insert(entity);
			}
		} else {
			this.entities.push(entity);
			if (this.level > 0 && this.entities.length > 8) {
				this.split();
			}
		}
	}

	public intersects(minX: number, minY: number, maxX: number, maxY: number): boolean {
		return minX < this.maxX && maxX > this.minX && minY < this.maxY && maxY > this.minY;
	}

	public query(minX: number, minY: number, maxX: number, maxY: number, result: Set<Entity> = new Set()): Set<Entity> {
		if (maxX <= this.minX || minX >= this.maxX || maxY <= this.minY || minY >= this.maxY) {
			return result;
		}
		if (this.hasChildren) {
			this.childTopLeft!.query(minX, minY, maxX, maxY, result);
			this.childTopRight!.query(minX, minY, maxX, maxY, result);
			this.childBottomLeft!.query(minX, minY, maxX, maxY, result);
			this.childBottomRight!.query(minX, minY, maxX, maxY, result);
		} else {
			const entitiesCount: number = this.entities.length;
			if (entitiesCount !== 0) {
				for (let i: number = 0; i < entitiesCount; i++) {
					const entity = this.entities[i];
					if (entity.minX < maxX && entity.maxX > minX && entity.minY < maxY && entity.maxY > minY) {
						result.add(entity);
					}
				}
			}
		}
		return result;
	}

	public clear(): void {
		this.childTopLeft = null;
		this.childTopRight = null;
		this.childBottomLeft = null;
		this.childBottomRight = null;
		this.hasChildren = false;
		this.entities.length = 0;
	}
}