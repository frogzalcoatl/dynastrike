import { Entity } from "../entity/entity";
import { computePenetrationAndNormal, ContactResult } from "./modules/computePenetrationAndNormal";
import { isPointInPolygon } from "./modules/isPointInPolygon";
import { Projection, projectPointOnEdge } from "./modules/projectPointOnEdge";

export class Collision {
	public static collideCircleCircle(instance: Entity, other: Entity): void {
		const distanceX: number = other.position.x - instance.position.x;
		const distanceY: number = other.position.y - instance.position.y;
		const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
		const minimumDistance: number = instance.radius + other.radius;
		if (distanceSquared >= minimumDistance * minimumDistance || (instance.isStatic && other.isStatic)) {
			return;
		}
		const distance: number = Math.sqrt(distanceSquared);
		let normalX: number = 0;
		let normalY: number = 0;
		if (distance < 1e-9) {
			const angle: number = Math.PI * 2 * Math.random();
			normalX = Math.cos(angle);
			normalY = Math.sin(angle);
		} else {
			const inverseDistance: number = 1 / distance;
			normalX = distanceX * inverseDistance;
			normalY = distanceY * inverseDistance;
		}
		const overlap: number = minimumDistance - distance;
		if (instance.isStatic) {
			other.moveBy(normalX * overlap, normalY * overlap);
		} else if (other.isStatic) {
			instance.moveBy(-normalX * overlap, -normalY * overlap);
		} else {
			const inverseTotalMass: number = 1 / (instance.mass + other.mass);
			const instanceCorrectionFraction: number = other.mass * inverseTotalMass;
			const otherCorrectionFraction: number = instance.mass * inverseTotalMass;
			instance.moveBy(-normalX * overlap * instanceCorrectionFraction, -normalY * overlap * instanceCorrectionFraction);
			other.moveBy(normalX * overlap * otherCorrectionFraction, normalY * overlap * otherCorrectionFraction);
		}
		const relativeVelocityX: number = other.velocity.x - instance.velocity.x;
		const relativeVelocityY: number = other.velocity.y - instance.velocity.y;
		const velocityAlongNormal: number = relativeVelocityX * normalX + relativeVelocityY * normalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const instanceInverseMass: number = instance.isStatic ? 0 : (instance.mass === 0 ? 0 : 1 / instance.mass);
		const otherInverseMass: number = other.isStatic ? 0 : (other.mass === 0 ? 0 : 1 / other.mass);
		const impulseMagnitude: number = -2 * velocityAlongNormal / (instanceInverseMass + otherInverseMass);
		const impulseX: number = impulseMagnitude * normalX;
		const impulseY: number = impulseMagnitude * normalY;
		if (!instance.isStatic) {
			instance.velocity.x -= impulseX * instanceInverseMass;
			instance.velocity.y -= impulseY * instanceInverseMass;
		}
		if (!other.isStatic) {
			other.velocity.x += impulseX * otherInverseMass;
			other.velocity.y += impulseY * otherInverseMass;
		}
	}

	public static collidePolygonCircle(polygonEntity: Entity, circleEntity: Entity): void {
		if (polygonEntity.points === null || (polygonEntity.isStatic && circleEntity.isStatic)) {
			return;
		}
		const circleRadiusSquared = circleEntity.radius * circleEntity.radius;
		const isInside: boolean = isPointInPolygon(circleEntity.position.x, circleEntity.position.y, polygonEntity.points);
		let sumPenetration: number = 0;
		let accumulatedNormalXTimesPenetration: number = 0;
		let accumulatedNormalYTimesPenetration: number = 0;
		let accumulatedContactXTimesPenetration: number = 0;
		let accumulatedContactYTimesPenetration: number = 0;
		let contactCount: number = 0;
		let minDistanceSqForInside: number = Infinity;
		let normalXForInside: number = 0;
		let normalYForInside: number = 0;
		let previousX: number = polygonEntity.points[polygonEntity.points.length - 2];
		let previousY: number = polygonEntity.points[polygonEntity.points.length - 1];
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const currentX: number = polygonEntity.points[i];
			const currentY: number = polygonEntity.points[i + 1];
			const projection: Projection = projectPointOnEdge(circleEntity.position.x, circleEntity.position.y, previousX, previousY, currentX, currentY);
			const contact: ContactResult = computePenetrationAndNormal(circleEntity.position.x, circleEntity.position.y, projection.x, projection.y, circleEntity.radius, polygonEntity.position.x, polygonEntity.position.y, isInside, projection.distanceSquared, circleRadiusSquared);
			if (contact.penetration > 0) {
				sumPenetration += contact.penetration;
				accumulatedNormalXTimesPenetration += contact.normalX * contact.penetration;
				accumulatedNormalYTimesPenetration += contact.normalY * contact.penetration;
				accumulatedContactXTimesPenetration += projection.x * contact.penetration;
				accumulatedContactYTimesPenetration += projection.y * contact.penetration;
				contactCount++;
			}
			if (isInside) {
				if (projection.distanceSquared < minDistanceSqForInside) {
					minDistanceSqForInside = projection.distanceSquared;
					normalXForInside = projection.x - circleEntity.position.x;
					normalYForInside = projection.y - circleEntity.position.y;
				}
			}
			previousX = currentX;
			previousY = currentY;
		}
		for (let i: number = 0; i < polygonEntity.points.length; i += 2) {
			const vertexX: number = polygonEntity.points[i];
			const vertexY: number = polygonEntity.points[i + 1];
			const distanceX: number = circleEntity.position.x - vertexX;
			const distanceY: number = circleEntity.position.y - vertexY;
			const distanceSquared: number = distanceX * distanceX + distanceY * distanceY;
			const contact: ContactResult = computePenetrationAndNormal(circleEntity.position.x, circleEntity.position.y, vertexX, vertexY, circleEntity.radius, polygonEntity.position.x, polygonEntity.position.y, isInside, distanceSquared, circleRadiusSquared);
			if (contact.penetration > 0) {
				sumPenetration += contact.penetration;
				accumulatedNormalXTimesPenetration += contact.normalX * contact.penetration;
				accumulatedNormalYTimesPenetration += contact.normalY * contact.penetration;
				accumulatedContactXTimesPenetration += vertexX * contact.penetration;
				accumulatedContactYTimesPenetration += vertexY * contact.penetration;
				contactCount++;
			}
			if (isInside) {
				if (distanceSquared < minDistanceSqForInside) {
					minDistanceSqForInside = distanceSquared;
					normalXForInside = vertexX - circleEntity.position.x;
					normalYForInside = vertexY - circleEntity.position.y;
				}
			}
		}
		if (contactCount === 0 && isInside) {
			if (minDistanceSqForInside === Infinity) {
				return;
			}
			const distToBoundary: number = Math.sqrt(minDistanceSqForInside);
			let bestNormalX: number = 0;
			let bestNormalY: number = 0;
			if (distToBoundary < 1e-9) {
				bestNormalX = 0;
				bestNormalY = 1;
			} else {
				const inverseDistance: number = 1 / distToBoundary;
				bestNormalX = normalXForInside * inverseDistance;
				bestNormalY = normalYForInside * inverseDistance;
			}
			const penetration: number = circleEntity.radius + distToBoundary;
			if (polygonEntity.isStatic) {
				circleEntity.moveBy(bestNormalX * penetration, bestNormalY * penetration);
			} else if (circleEntity.isStatic) {
				polygonEntity.moveBy(-bestNormalX * penetration, -bestNormalY * penetration);
			} else {
				const inverseTotalMass = 1 / (polygonEntity.mass + circleEntity.mass);
				const polygonCorrectionFactor = circleEntity.mass * inverseTotalMass;
				const circleCorrectionFactor = polygonEntity.mass * inverseTotalMass;
				polygonEntity.moveBy(-bestNormalX * penetration * polygonCorrectionFactor, -bestNormalY * penetration * polygonCorrectionFactor);
				circleEntity.moveBy(bestNormalX * penetration * circleCorrectionFactor, bestNormalY * penetration * circleCorrectionFactor);
			}
			return;
		}
		if (contactCount === 0) {
			return;
		}
		let weightedNormalX: number = 0;
		let weightedNormalY: number = 0;
		let averageContactX: number = 0;
		let averageContactY: number = 0;
		if (sumPenetration < 1e-9) {
			weightedNormalX = 0;
			weightedNormalY = 0;
			averageContactX = 0;
			averageContactY = 0;
		} else {
			const inverseSum: number = 1 / sumPenetration;
			weightedNormalX = accumulatedNormalXTimesPenetration * inverseSum;
			weightedNormalY = accumulatedNormalYTimesPenetration * inverseSum;
			averageContactX = accumulatedContactXTimesPenetration * inverseSum;
			averageContactY = accumulatedContactYTimesPenetration * inverseSum;
		}
		const averageNormalLength: number = Math.sqrt(weightedNormalX * weightedNormalX + weightedNormalY * weightedNormalY);
		let finalNormalX: number = 0;
		let finalNormalY: number = 0;
		if (averageNormalLength < 1e-9) {
			finalNormalX = 0;
			finalNormalY = 1;
		} else {
			const inverseLength: number = 1 / averageNormalLength;
			finalNormalX = weightedNormalX * inverseLength;
			finalNormalY = weightedNormalY * inverseLength;
		}
		const correctionMagnitude: number = (sumPenetration / contactCount);
		if (polygonEntity.isStatic) {
			circleEntity.moveBy(finalNormalX * correctionMagnitude, finalNormalY * correctionMagnitude);
		} else if (circleEntity.isStatic) {
			polygonEntity.moveBy(-finalNormalX * correctionMagnitude, -finalNormalY * correctionMagnitude);
		} else {
			const totalMass = polygonEntity.mass + circleEntity.mass;
			const inverseTotalMass = 1 / totalMass;
			const polygonCorrectionFactor = circleEntity.mass * inverseTotalMass;
			const circleCorrectionFactor = polygonEntity.mass * inverseTotalMass;
			polygonEntity.moveBy(-finalNormalX * correctionMagnitude * polygonCorrectionFactor, -finalNormalY * correctionMagnitude * polygonCorrectionFactor);
			circleEntity.moveBy(finalNormalX * correctionMagnitude * circleCorrectionFactor, finalNormalY * correctionMagnitude * circleCorrectionFactor);
		}
		const relativePolygonX: number = averageContactX - polygonEntity.position.x;
		const relativePolygonY: number = averageContactY - polygonEntity.position.y;
		const relativeCircleX: number = averageContactX - circleEntity.position.x;
		const relativeCircleY: number = averageContactY - circleEntity.position.y;
		const relativeVelocityX: number = (circleEntity.velocity.x - circleEntity.angularVelocity * relativeCircleY) - (polygonEntity.velocity.x - polygonEntity.angularVelocity * relativePolygonY);
		const relativeVelocityY: number = (circleEntity.velocity.y + circleEntity.angularVelocity * relativeCircleX) - (polygonEntity.velocity.y + polygonEntity.angularVelocity * relativePolygonX);
		const velocityAlongNormal: number = relativeVelocityX * finalNormalX + relativeVelocityY * finalNormalY;
		if (velocityAlongNormal > 0) {
			return;
		}
		const inversePolygonMass: number = polygonEntity.isStatic ? 0 : 1 / polygonEntity.mass;
		const inverseCircleMass: number = circleEntity.isStatic ? 0 : 1 / circleEntity.mass;
		let invInertiaPoly: number = 0;
		if (!polygonEntity.isStatic && polygonEntity.inertia > 1e-9) {
			invInertiaPoly = 1 / polygonEntity.inertia;
		}
		let invInertiaCirc: number = 0;
		if (!circleEntity.isStatic && circleEntity.inertia > 1e-9) {
			invInertiaCirc = 1 / circleEntity.inertia;
		}
		const relativePolygonPerpendicular: number = relativePolygonX * finalNormalY - relativePolygonY * finalNormalX;
		const relativeCirclePerpendicular: number = relativeCircleX * finalNormalY - relativeCircleY * finalNormalX;
		const denominator: number = inversePolygonMass + inverseCircleMass + relativePolygonPerpendicular * relativePolygonPerpendicular * invInertiaPoly + relativeCirclePerpendicular * relativeCirclePerpendicular * invInertiaCirc;
		if (denominator < 1e-9) {
			return;
		}
		const impulseMagnitude: number = (-2.0 * velocityAlongNormal) / denominator;
		const impulseX: number = impulseMagnitude * finalNormalX;
		const impulseY: number = impulseMagnitude * finalNormalY;
		if (!polygonEntity.isStatic) {
			polygonEntity.velocity.x -= impulseX * inversePolygonMass;
			polygonEntity.velocity.y -= impulseY * inversePolygonMass;
			polygonEntity.angularVelocity -= (relativePolygonX * impulseY - relativePolygonY * impulseX) * invInertiaPoly;
		}
		if (!circleEntity.isStatic) {
			circleEntity.velocity.x += impulseX * inverseCircleMass;
			circleEntity.velocity.y += impulseY * inverseCircleMass;
			circleEntity.angularVelocity += (relativeCircleX * impulseY - relativeCircleY * impulseX) * invInertiaCirc;
		}
	}

	public static collidePolygonPolygon(instance: Entity, other: Entity): void {
		if ((instance.points === null || other.points === null) || instance.isStatic && other.isStatic) {
			return;
		}
	}

	public static collide(instance: Entity, other: Entity): void {
		if (instance.points === null && other.points === null) {
			this.collideCircleCircle(instance, other);
			return;
		}
		if (instance.points !== null && other.points === null) {
			this.collidePolygonCircle(instance, other);
			return;
		}
		if (instance.points === null && other.points !== null) {
			this.collidePolygonCircle(other, instance);
			return;
		}
		if (instance.points !== null && other.points !== null) {
			this.collidePolygonPolygon(instance, other);
			return;
		}
	}
}