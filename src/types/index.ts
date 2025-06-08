import { QuadTree } from "../spatial/quadtree";

export type BoundaryBuffer = number[];

export interface Circle {
	x: number;
	y: number;
	radius: number;
}

export interface Box {
	minX: number;
	minY: number;
	maxX: number;
	maxY: number;
}

export interface ClosestPoint {
	x: number;
	y: number;
	distanceSquared: number;
}

export interface ProjectionRange {
	minimum: number;
	maximum: number
}

export interface Projection {
	x: number;
	y: number;
	distanceSquared: number;
}

export interface QuadTreeChildren {
	topLeft: QuadTree;
	topRight: QuadTree;
	bottomLeft: QuadTree;
	bottomRight: QuadTree;
}

export interface ContactPoints {
	x: number;
	y: number;
	count: number
}

export interface SerializedEntity {
	positionX: number;
	positionY: number;
	velocityX: number;
	velocityY: number;
	radius: number;
	points: number[] | null;
	angle: number;
	angularVelocity: number;
	mass: number;
	isStatic: boolean;
	frictionCoefficient: number;
	restitution: number;
	linearDampingFactor: number;
	angularDampingFactor: number;
}

export interface IJoint {
	update(deltaTime: number): void;
}